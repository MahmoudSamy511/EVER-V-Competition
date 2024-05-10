#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import csv
import os
from datetime import datetime
from straight_line import straight_line

class OdomImuLogger:
    def __init__(self):
        self.odom_data = []
        self.imu_data = []

        # Initialize ROS node
        rospy.init_node('odom_logger', anonymous=True)

        # Subscribe to odom topic
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        odom_values = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.odom_data.append(odom_values)

    def save_to_csv(self):
        # Create directory for saving CSV files if not exists
        if not os.path.exists("data"):
            os.makedirs("data")

        # Write odom data to CSV
        with open("data/odom_data.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["X", "Y"])
            writer.writerows(self.odom_data)

def main():
    logger = OdomImuLogger()
    straight_line()
    logger.save_to_csv()

if __name__ == '__main__':
    main()