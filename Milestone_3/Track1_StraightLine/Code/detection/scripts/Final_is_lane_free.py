#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class LaneAndPavementDetection:
    def __init__(self):
        rospy.init_node('lane_and_pavement_detection', anonymous=True)
        self.lidar_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.lidar_callback)
        
        # Define the publishers
        self.left_lane_free_pub = rospy.Publisher('/is_left_lane_free', Bool, queue_size=10)
        self.right_lane_free_pub = rospy.Publisher('/is_right_lane_free', Bool, queue_size=10)
        
        self.left_lane_available = True
        self.right_lane_available = True
        self.left_pavement_detected = False
        self.right_pavement_detected = False
        self.MIN_POINTS_FOR_LANE = 10  # Adjust as needed
        self.MIN_POINTS_FOR_PAVEMENT = 10  # Adjust as needed

    def lidar_callback(self, data):
        left_lane_points = 0
        right_lane_points = 0
        left_pavement_points = 0
        right_pavement_points = 0

        for point in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point

            # Lane detection conditions
            if -2 <= y <= 50 and 2.5 < x <= 5 and -1.4 <= z:
                left_lane_points += 1
            elif -2 <= y <= 50 and -5 <= x < -2.5 and -1.4 <= z:
                right_lane_points += 1

            # Pavement detection conditions
            if y <= 8 and -3.8 <= x < -2 and -1.4 <= z <= -1.2:
                left_pavement_points += 1
            elif y <= 8 and 2 < x <= 3.8 and -1.4 <= z <= -1.2:
                right_pavement_points += 1

        # Lane availability logic
        if left_lane_points == 0 and right_lane_points == 0:
            self.left_lane_available = True
            self.right_lane_available = True
        else:
            self.left_lane_available = left_lane_points < self.MIN_POINTS_FOR_LANE
            self.right_lane_available = right_lane_points < self.MIN_POINTS_FOR_LANE

        # Flip the logic for left and right lanes
        self.left_lane_available, self.right_lane_available = self.right_lane_available, self.left_lane_available

        # Pavement detection logic
        self.left_pavement_detected = left_pavement_points >= self.MIN_POINTS_FOR_PAVEMENT
        self.right_pavement_detected = right_pavement_points >= self.MIN_POINTS_FOR_PAVEMENT

        # Determine if lanes are free
        is_left_lane_free = self.left_lane_available and not self.left_pavement_detected
        is_right_lane_free = self.right_lane_available and not self.right_pavement_detected

        # Log lane availability and pavement detection status
        # rospy.loginfo(f"is_left_lane_free: {is_left_lane_free}")
        # rospy.loginfo(f"is_right_lane_free: {is_right_lane_free}")

        # Publish the lane status
        self.left_lane_free_pub.publish(Bool(is_left_lane_free))
        self.right_lane_free_pub.publish(Bool(is_right_lane_free))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LaneAndPavementDetection()
        node.run()
    except rospy.ROSInterruptException:
        pass