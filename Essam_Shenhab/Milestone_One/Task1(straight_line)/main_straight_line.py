#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import csv
import os
from datetime import datetime
from straight_line import straight_line

class OdomImuLogger:
    def __init__(self):
        self.odom_data = []
        self.imu_data = []

        # Initialize ROS node
        rospy.init_node('odom_imu_logger', anonymous=True)

        # Subscribe to odom topic
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Subscribe to imu topic
        rospy.Subscriber('/Imu', Imu, self.imu_callback)

    def odom_callback(self, msg):
        # Process and store odometry data
        time = datetime.fromtimestamp(rospy.Time.now().to_sec()).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        odom_values = [time, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.odom_data.append(odom_values)


    def imu_callback(self, msg):
        # Process and store IMU data
        time = datetime.fromtimestamp(rospy.Time.now().to_sec()).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        imu_values = [time, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.y, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        self.imu_data.append(imu_values)

    def save_to_csv(self):
        # Create directory for saving CSV files if not exists
        if not os.path.exists("data"):
            os.makedirs("data")

        # Write odom data to CSV
        with open("data/odom_data.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "X", "Y", "Z"])
            writer.writerows(self.odom_data)

        # Write imu data to CSV
        with open("data/imu_data.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "Orientation_X", "Orientation_Y", "Orientation_Z", "Angular_Velocity_X", "Angular_Velocity_Y", "Angular_Velocity_Z", "Linear_Acceleration_X", "Linear_Acceleration_Y", "Linear_Acceleration_Z"])
            writer.writerows(self.imu_data)

def main():
    logger = OdomImuLogger()
    straight_line()
    logger.save_to_csv()

if __name__ == '__main__':
    main()