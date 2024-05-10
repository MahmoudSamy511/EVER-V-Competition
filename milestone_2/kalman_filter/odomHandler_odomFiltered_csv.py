#!/usr/bin/env python

import rospy
import csv
from nav_msgs.msg import Odometry

class OdometryReader:
    def __init__(self):
        rospy.init_node('odometry_reader', anonymous=True)
        rospy.Subscriber('/filtered_odom', Odometry, self.odom_callback)
        self.csv_file = open('filtered_odom_positions.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time', 'Position_X', 'Position_Y'])

    def odom_callback(self, msg):
        time = msg.header.stamp
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y

        self.csv_writer.writerow([time, position_x, position_y])

    def run(self):
        rospy.spin()

    def __del__(self):
        self.csv_file.close()

if __name__ == '__main__':
    reader = OdometryReader()
    reader.run()
