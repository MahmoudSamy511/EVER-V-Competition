#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import cvlib as cv
import tensorflow as tf
import numpy as np
import json


class ImageSubscriber:
    def __init__(self):
        rospy.init_node('object_detection_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image", Image, self.image_callback)
        self.label_pub = rospy.Publisher('/detected_object_labels', String, queue_size=10)
        self.window_name = 'Object Detection'
        self.color_green = (0, 255, 0)
        self.color_blue = (255, 0, 0)

        # Ensure TensorFlow uses the CPU only
        gpus = tf.config.experimental.list_physical_devices('GPU')
        if gpus:
            try:
                tf.config.experimental.set_visible_devices([], 'GPU')
            except RuntimeError as e:
                print(e)

        # Dictionary to store highest priority object for each lane
        self.lane_objects = {"left lane": None, "center": None, "right lane": None}

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.detect_objects(cv_image)
            self.detect_cones(cv_image)
            self.publish_lane_objects()
            self.draw_info(cv_image)
        except Exception as e:
            rospy.logerr(f"Image callback error: {e}")

    def get_position(self, x, width):
        # Divide the image into three parts: left, center, right
        left_boundary = width // 3
        right_boundary = 2 * (width // 3)

        if x < left_boundary:
            return "left lane"
        elif x < right_boundary:
            return "center"
        else:
            return "right lane"

    def detect_objects(self, cv_image):
        bbox, label, conf = cv.detect_common_objects(cv_image)
        height, width, _ = cv_image.shape

        for i in range(len(bbox)):
            text = f"{label[i]}: {conf[i]:.2f}"
            x, y, w, h = bbox[i]

            # Draw a bounding box
            cv2.rectangle(cv_image, (x, y), (w, h), self.color_green, 2)

            # Draw a plus sign at the centroid
            centroid_x = int((x + w) / 2)
            centroid_y = int((y + h) / 2)
            plus_size = 10
            cv2.line(cv_image, (centroid_x - plus_size, centroid_y), (centroid_x + plus_size, centroid_y),
                     self.color_green, 2)
            cv2.line(cv_image, (centroid_x, centroid_y - plus_size), (centroid_x, centroid_y + plus_size),
                     self.color_green, 2)

            # Add text with the label and confidence
            cv2.putText(cv_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_green, 2)

            # Determine position
            position = self.get_position(centroid_x, width)
            
            # Update lane_objects dictionary
            if label[i] == "person":
                self.lane_objects[position] = "person"
            elif self.lane_objects[position] is None:
                self.lane_objects[position] = label[i]

    def detect_cones(self, cv_image):
        # Convert to RGB
        img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Convert to HSV
        img_HSV = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)

        # Thresholding to detect cones
        img_thresh_low = cv2.inRange(img_HSV, np.array([0, 135, 135]), np.array([15, 255, 255]))
        img_thresh_high = cv2.inRange(img_HSV, np.array([159, 135, 135]), np.array([179, 255, 255]))
        img_thresh = cv2.bitwise_or(img_thresh_low, img_thresh_high)

        # Morphological opening
        kernel = np.ones((5, 5))
        img_thresh_opened = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)

        # Median blur
        img_thresh_blurred = cv2.medianBlur(img_thresh_opened, 5)

        # Edge detection
        img_edges = cv2.Canny(img_thresh_blurred, 80, 160)

        # Find contours
        contours, _ = cv2.findContours(np.array(img_edges), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Approximate contours
        approx_contours = [cv2.approxPolyDP(c, 10, closed=True) for c in contours]

        # Compute convex hulls
        all_convex_hulls = [cv2.convexHull(ac) for ac in approx_contours]

        # Filter convex hulls with 3 to 10 points
        convex_hulls_3to10 = [ch for ch in all_convex_hulls if 3 <= len(ch) <= 10]

        # Define function to check if a convex hull is pointing up
        def convex_hull_pointing_up(ch):
            points_above_center, points_below_center = [], []

            x, y, w, h = cv2.boundingRect(ch)
            aspect_ratio = w / h
            if aspect_ratio < 0.8:
                vertical_center = y + h / 2

                for point in ch:
                    if point[0][1] < vertical_center:
                        points_above_center.append(point)
                    elif point[0][1] >= vertical_center:
                        points_below_center.append(point)
                left_x = points_below_center[0][0][0]
                right_x = points_below_center[0][0][0]
                for point in points_below_center:
                    if point[0][0] < left_x:
                        left_x = point[0][0]
                    if point[0][0] > right_x:
                        right_x = point[0][0]

                for point in points_above_center:
                    if (point[0][0] < left_x) or (point[0][0] > right_x):
                        return False
            else:
                return False

            return True

        # Detect cones and their bounding rectangles
        height, width, _ = cv_image.shape
        cones = []
        bounding_rects = []
        for ch in convex_hulls_3to10:
            if convex_hull_pointing_up(ch):
                cones.append(ch)
                rect = cv2.boundingRect(ch)
                bounding_rects.append(rect)

        # Draw detected cones on the original image
        cv2.drawContours(cv_image, cones, -1, (255, 255, 255), 2)
        for rect in bounding_rects:
            x, y, w, h = rect
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            centroid_x = x + w // 2
            centroid_y = y + h // 2
            cv2.line(cv_image, (centroid_x - 10, centroid_y), (centroid_x + 10, centroid_y), self.color_blue, 2)
            cv2.line(cv_image, (centroid_x, centroid_y - 10), (centroid_x, centroid_y + 10), self.color_blue, 2)
            cv2.putText(cv_image, "Cone", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_blue, 2)
            
            # Determine position
            position = self.get_position(centroid_x, width)
            
            # Update lane_objects dictionary
            if self.lane_objects[position] is None:
                self.lane_objects[position] = "cone"

    def publish_lane_objects(self):
        # Create a dictionary to publish
        publish_dict = {}
        for position in self.lane_objects.keys():
            label = self.lane_objects[position]
            if label is None:
                publish_dict[position] = "none"
            else:
                publish_dict[position] = label

        # Convert dictionary to JSON string and publish
        publish_str = json.dumps(publish_dict)
        # rospy.loginfo(publish_str)
        self.label_pub.publish(publish_str)

        # Reset the lane_objects dictionary for the next frame
        self.lane_objects = {"left lane": None, "center": None, "right lane": None}

    def draw_info(self, image):
        cv2.imshow(self.window_name, image)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        image_subscriber = ImageSubscriber()
        image_subscriber.run()
    except rospy.ROSInterruptException:
        pass
