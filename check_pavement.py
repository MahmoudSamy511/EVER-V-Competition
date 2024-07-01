#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class PavementDetection:
    def __init__(self):
        rospy.init_node('pavement_detection', anonymous=True)
        self.lidar_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.lidar_callback)
        self.left_pavement_detected = False
        self.right_pavement_detected = False
        self.MIN_POINTS_FOR_PAVEMENT = 10  # Adjust as needed

    def lidar_callback(self, data):
        left_pavement_points = 0
        right_pavement_points = 0

        for point in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point

            # Define conditions for detecting right (formerly left) and left (formerly right) pavement
            if -9 <= y <= 9 and -4 <= x < -1 and -1.55 <= z <= -1.2:
                left_pavement_points += 1
            elif -9 <= y <= 9 and 1 < x <= 4 and -1.55 <= z <= -1.2:
                right_pavement_points += 1

        # Update pavement detection status based on point count
        self.left_pavement_detected = left_pavement_points >= self.MIN_POINTS_FOR_PAVEMENT
        self.right_pavement_detected = right_pavement_points >= self.MIN_POINTS_FOR_PAVEMENT

        # Log pavement detection status
        rospy.loginfo(f"Pavement on Left: {self.left_pavement_detected}")
        rospy.loginfo(f"Pavement on Right: {self.right_pavement_detected}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PavementDetection()
        node.run()
    except rospy.ROSInterruptException:
        pass
