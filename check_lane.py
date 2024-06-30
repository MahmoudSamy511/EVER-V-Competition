#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class LaneDetection:
    def __init__(self):
        rospy.init_node('lane_detection', anonymous=True)
        self.lidar_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.lidar_callback)
        self.left_lane_available = True
        self.right_lane_available = True
        self.MIN_POINTS_FOR_LANE = 10  # Adjust as needed

    def lidar_callback(self, data):
        left_lane_points = 0
        right_lane_points = 0
        
        for point in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
            y, x, z = point
            if -5 <= x <= 5 and 1 < y <= 5:
                left_lane_points += 1
            elif -5 <= x <= 5 and -5 <= y < -1:
                right_lane_points += 1

        if left_lane_points == 0 and right_lane_points == 0:
            self.left_lane_available = True
            self.right_lane_available = True
        else:
            self.left_lane_available = left_lane_points < self.MIN_POINTS_FOR_LANE
            self.right_lane_available = right_lane_points < self.MIN_POINTS_FOR_LANE

        # Flip the logic for left and right lanes
        self.left_lane_available, self.right_lane_available = self.right_lane_available, self.left_lane_available

        rospy.loginfo(f"Left lane available: {self.left_lane_available}")
        rospy.loginfo(f"Right lane available: {self.right_lane_available}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LaneDetection()
        node.run()
    except rospy.ROSInterruptException:
        pass
