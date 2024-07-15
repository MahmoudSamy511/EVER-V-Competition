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
        self.person_flag = 0

        # Ensure TensorFlow uses the CPU only
        gpus = tf.config.experimental.list_physical_devices('GPU')
        if gpus:
            try:
                tf.config.experimental.set_visible_devices([], 'GPU')
            except RuntimeError as e:
                print(e)

        # Dictionary to store highest priority object for each lane
        self.lane_objects = {"left lane": None, "center": None, "right lane": None}
        self.left_line = None
        self.right_line = None

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.detect_lanes(cv_image)
            self.detect_objects(cv_image)
            self.detect_cones(cv_image)
            self.publish_lane_objects()
            self.draw_info(cv_image)
        except Exception as e:
            rospy.logerr(f"Image callback error: {e}")

    def get_position(self, x, y, width, h):
        if self.left_line is None or self.right_line is None:
            left_boundary = width // 3
            right_boundary = 2 * (width // 3)

            if x < left_boundary:
                return "left lane"
            elif x < right_boundary:
                return "center"
            else:
                return "right lane"
        else:
            def is_left_of_line(line, x, y):
                x1, y1 = line[0]
                x2, y2 = line[1]
                return (y2 - y1) * (x - x1) - (x2 - x1) * (y - y1) > 0

            lower_y = y + h
            left_of_left_line = is_left_of_line(self.left_line, x, h)
            right_of_right_line = not is_left_of_line(self.right_line, x, h)


            if left_of_left_line:
                return "left lane"
            elif right_of_right_line:
                return "right lane"
            else:
                return "center"

    def detect_objects(self, cv_image):
        bbox, label, conf = cv.detect_common_objects(cv_image)
        height, width, _ = cv_image.shape

        for i in range(len(bbox)):
            text = f"{label[i]}: {conf[i]:.2f}"
            x, y, w, h = bbox[i]

            cv2.rectangle(cv_image, (x, y), (w, h), self.color_green, 2)

            centroid_x = int((x + w) / 2)
            centroid_y = int((y + h) / 2)
            plus_size = 10
            cv2.line(cv_image, (centroid_x - plus_size, centroid_y), (centroid_x + plus_size, centroid_y),
                    self.color_blue, 2)
            cv2.line(cv_image, (centroid_x, centroid_y - plus_size), (centroid_x, centroid_y + plus_size),
                    self.color_blue, 2)


            cv2.putText(cv_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_green, 2)

            position = self.get_position(centroid_x, centroid_y, width, h)

            if label[i] == "person":
                self.person_flag = 1
                self.lane_objects[position] = "person"
                # rospy.loginfo(centroid_y)
            elif self.lane_objects[position] is None:
                self.lane_objects[position] = label[i]

    def detect_cones(self, cv_image):
        img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img_HSV = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)

        img_thresh_low = cv2.inRange(img_HSV, np.array([0, 135, 135]), np.array([15, 255, 255]))
        img_thresh_high = cv2.inRange(img_HSV, np.array([159, 135, 135]), np.array([179, 255, 255]))
        img_thresh = cv2.bitwise_or(img_thresh_low, img_thresh_high)

        kernel = np.ones((5, 5))
        img_thresh_opened = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
        img_thresh_blurred = cv2.medianBlur(img_thresh_opened, 5)
        img_edges = cv2.Canny(img_thresh_blurred, 80, 160)

        contours, _ = cv2.findContours(np.array(img_edges), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        approx_contours = [cv2.approxPolyDP(c, 10, closed=True) for c in contours]
        all_convex_hulls = [cv2.convexHull(ac) for ac in approx_contours]
        convex_hulls_3to10 = [ch for ch in all_convex_hulls if 3 <= len(ch) <= 10]

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

        height, width, _ = cv_image.shape
        cones = []
        bounding_rects = []
        for ch in convex_hulls_3to10:
            if convex_hull_pointing_up(ch):
                cones.append(ch)
                rect = cv2.boundingRect(ch)
                bounding_rects.append(rect)

        cv2.drawContours(cv_image, cones, -1, (255, 255, 255), 2)
        for rect in bounding_rects:
            x, y, w, h = rect
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            centroid_x = x + w // 2
            centroid_y = y + h // 2
            cv2.line(cv_image, (centroid_x - 10, centroid_y), (centroid_x + 10, centroid_y), self.color_blue, 2)
            cv2.line(cv_image, (centroid_x, centroid_y - 10), (centroid_x, centroid_y + 10), self.color_blue, 2)
            cv2.putText(cv_image, "Cone", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_blue, 2)

            position = self.get_position(centroid_x, centroid_y, width, h)

            if self.lane_objects[position] is None:
                self.lane_objects[position] = "cone"

    def publish_lane_objects(self):
        rate = rospy.Rate(3)
        if self.lane_objects["center"] != None and self.lane_objects["center"] != "person":
            self.person_flag = 0

        if self.person_flag:
            lane_objects_dict = {
            "left lane": self.lane_objects["left lane"] if self.lane_objects["left lane"] is not None else "none",
            "center": "person",
            "right lane": self.lane_objects["right lane"] if self.lane_objects["right lane"] is not None else "none"
            }
        else:  
            lane_objects_dict = {
                "left lane": self.lane_objects["left lane"] if self.lane_objects["left lane"] is not None else "none",
                "center": self.lane_objects["center"] if self.lane_objects["center"] is not None else "none",
                "right lane": self.lane_objects["right lane"] if self.lane_objects["right lane"] is not None else "none"
            }
        

        # Convert the dictionary to a JSON string
        lane_objects_str = json.dumps(lane_objects_dict)

        # Publish the JSON string
        self.label_pub.publish(lane_objects_str)

        # Reset the lane objects
        self.lane_objects = {"left lane": None, "center": None, "right lane": None}
        rate.sleep()

    def draw_info(self, image):
        # cv2.imshow(self.window_name, image)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

    # Lane detection functions
    def convert_hsl(self, image):
        return cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

    def HSL_color_selection(self, image):
        converted_image = self.convert_hsl(image)

        lower_threshold = np.uint8([0, 200, 0])
        upper_threshold = np.uint8([255, 255, 255])
        white_mask = cv2.inRange(converted_image, lower_threshold, upper_threshold)

        lower_threshold = np.uint8([10, 0, 100])
        upper_threshold = np.uint8([40, 255, 255])
        yellow_mask = cv2.inRange(converted_image, lower_threshold, upper_threshold)

        mask = cv2.bitwise_or(white_mask, yellow_mask)
        masked_image = cv2.bitwise_and(image, image, mask=mask)

        return masked_image

    def convert_gray_scale(self, image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    def gaussian_smoothing(self, image, kernel_size=13):
        return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

    def canny_detector(self, image, low_threshold, high_threshold):
        return cv2.Canny(image, low_threshold, high_threshold)

    def region_selection(self, image):
        mask = np.zeros_like(image)

        if len(image.shape) > 2:
            channel_count = image.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255

        rows, cols = image.shape[:2]

        bottom_left = [cols * 0, rows * 1]
        top_left = [cols * 0.3, rows * 0.6]
        bottom_right = [cols * 1, rows * 1]
        top_right = [cols * 0.7, rows * 0.6]

        vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        cv2.fillPoly(mask, vertices, ignore_mask_color)

        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    def hough_transform(self, image):
        rho = 1
        theta = np.pi / 180
        threshold = 20
        min_line_len = 20
        max_line_gap = 300

        return cv2.HoughLinesP(image, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)

    def draw_lines(self, image, lines, color=[0, 0, 255], thickness=2):
        line_image = np.zeros_like(image)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), color, thickness)

        return line_image

    def weighted_img(self, img, initial_img, α=0.8, β=1., γ=0.):
        return cv2.addWeighted(initial_img, α, img, β, γ)

    def average_slope_intercept(self, lines):
        left_lines = []  # (slope, intercept)
        left_weights = []  # (length,)
        right_lines = []  # (slope, intercept)
        right_weights = []  # (length,)

        for line in lines:
            for x1, y1, x2, y2 in line:
                if x2 == x1:
                    continue  # ignore a vertical line

                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - slope * x1
                length = np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)

                if slope < 0:  # y is reversed in image
                    left_lines.append((slope, intercept))
                    left_weights.append((length))
                else:
                    right_lines.append((slope, intercept))
                    right_weights.append((length))

        left_lane = np.dot(left_weights, left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
        right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None

        return left_lane, right_lane

    def make_line_points(self, y1, y2, line):
        if line is None:
            return None

        slope, intercept = line

        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        y1 = int(y1)
        y2 = int(y2)

        return ((x1, y1), (x2, y2))

    def lane_lines(self, image, lines):
        left_lane, right_lane = self.average_slope_intercept(lines)

        y1 = image.shape[0]
        y2 = y1 * 0.6

        left_line = self.make_line_points(y1, y2, left_lane)
        right_line = self.make_line_points(y1, y2, right_lane)

        return left_line, right_line

    def detect_lanes(self, cv_image):
        image = self.HSL_color_selection(cv_image)
        gray = self.convert_gray_scale(image)
        smooth_gray = self.gaussian_smoothing(gray)
        edges = self.canny_detector(smooth_gray, 50, 150)
        regions = self.region_selection(edges)
        lines = self.hough_transform(regions)

        if lines is not None:
            self.left_line, self.right_line = self.lane_lines(cv_image, lines)

            if self.left_line:
                cv2.line(cv_image, self.left_line[0], self.left_line[1], [255, 0, 0], 10)
            if self.right_line:
                cv2.line(cv_image, self.right_line[0], self.right_line[1], [0, 0, 255], 10)

        # cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        image_subscriber = ImageSubscriber()
        image_subscriber.run()
    except rospy.ROSInterruptException:
        pass