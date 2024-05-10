#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cvlib as cv
from cvlib.object_detection import draw_bbox
import numpy as np

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('object_detection_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image", Image, self.image_callback)
        self.window_name = 'Object Detection'
        self.color_green = (0, 255, 0)
        self.color_blue = (255, 0, 0)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.detect_objects(cv_image)
            self.detect_cones(cv_image)
            self.draw_info(cv_image)
        except Exception as e:
            print(e)

    def detect_objects(self, cv_image):
        bbox, label, conf = cv.detect_common_objects(cv_image)

        for i in range(len(bbox)):
            text = f"{label[i]}: {conf[i]:.2f}"
            x, y, w, h = bbox[i]

            # Draw a bounding box
            cv2.rectangle(cv_image, (x, y), (w, h), self.color_green, 2)

            # Draw a plus sign at the centroid
            centroid_x = int((x + w) / 2)
            centroid_y = int((y + h) / 2)
            plus_size = 10
            cv2.line(cv_image, (centroid_x - plus_size, centroid_y), (centroid_x + plus_size, centroid_y), self.color_green, 2)
            cv2.line(cv_image, (centroid_x, centroid_y - plus_size), (centroid_x, centroid_y + plus_size), self.color_green, 2)

            # Add text with the label and confidence
            cv2.putText(cv_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_green, 2)

    def detect_cones(self, cv_image):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([10, 100, 100])
        upper_orange = np.array([30, 255, 255])
        mask = cv2.inRange(hsv_image, lower_orange, upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Calculate contour area and bounding rectangle
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            confidence = area / (w * h)

            # Check if contour area is reasonable for a cone
            if area > 1000:
                # Draw bounding rectangle and centroid marker
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), self.color_blue, 2)
                centroid_x = x + w // 2
                centroid_y = y + h // 2
                cv2.line(cv_image, (centroid_x - 10, centroid_y), (centroid_x + 10, centroid_y), self.color_blue, 2)
                cv2.line(cv_image, (centroid_x, centroid_y - 10), (centroid_x, centroid_y + 10), self.color_blue, 2)
                
                # Add confidence as text
                cv2.putText(cv_image, f"Cone: {confidence:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_blue, 2)

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