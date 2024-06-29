#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import cvlib as cv
import numpy as np
from sensor_msgs import point_cloud2

class ImageLidarSubscriber:
    def __init__(self):
        rospy.init_node('image_lidar_subscriber', anonymous=True)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image", Image, self.image_callback)
        self.lidar_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.lidar_callback)

        self.image = None
        self.lidar_points = None

        # Dummy camera intrinsic parameters for example purposes
        self.fx, self.fy = 800, 800
        self.cx, self.cy = 640, 480
        # Dummy transformation matrix (identity matrix as placeholder)
        self.transformation_matrix = np.identity(4)

    def image_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.detect_objects()
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def lidar_callback(self, data):
        try:
            self.lidar_points = list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
        except Exception as e:
            rospy.logerr(f"Error processing lidar data: {e}")

    def detect_objects(self):
        if self.image is None or self.lidar_points is None:
            return

        bbox, label, conf = cv.detect_common_objects(self.image)
        lidar_points_transformed = self.transform_lidar_points()

        for i in range(len(bbox)):
            text = f"{label[i]}: {conf[i]:.2f}"
            x, y, w, h = bbox[i]
            centroid_x = int((x + w) / 2)
            centroid_y = int((y + h) / 2)

            distance = self.calculate_distance(centroid_x, centroid_y, lidar_points_transformed)
            if distance == float('inf'):
                rospy.loginfo(f"Object: {label[i]}, Distance: Not Found")
            else:
                rospy.loginfo(f"Object: {label[i]}, Distance: {distance:.2f} meters")

    def transform_lidar_points(self):
        # Transform lidar points to the camera frame
        transformed_points = []
        for point in self.lidar_points:
            x, y, z = point
            point_cam = np.dot(self.transformation_matrix, np.array([x, y, z, 1]).T)
            transformed_points.append(point_cam[:3])
        return transformed_points

    def calculate_distance(self, u, v, lidar_points):
        # Calculate the distance of the nearest lidar point to the pixel (u, v)
        min_distance = float('inf')
        for point in lidar_points:
            x, y, z = point
            u_proj, v_proj = self.project_point(x, y, z)
            if abs(u - u_proj) < 5 and abs(v - v_proj) < 5:
                distance = np.sqrt(x**2 + y**2 + z**2)
                if distance < min_distance:
                    min_distance = distance
        return min_distance

    def project_point(self, x, y, z):
        if z == 0:
            return -1, -1
        u = int(x * self.fx / z + self.cx)
        v = int(y * self.fy / z + self.cy)
        return u, v

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ImageLidarSubscriber()
        node.run()
    except rospy.ROSInterruptException:
        pass
