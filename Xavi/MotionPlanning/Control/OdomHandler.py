import os
import sys
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from csv import reader
from matplotlib import pyplot as plt
from tf.transformations import euler_from_quaternion

# Import your local modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../MotionPlanning/")
import Control.draw as draw
import CurvesGenerator.cubic_spline as cs


class OdometryHandler:
    """
    Class to handle odometry data.
    """
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        
        # Initialize the subscriber to receive odometry messages
        rospy.Subscriber('/odom', Odometry, self.log_callback_odom)

    def log_callback_odom(self, msg):
        """
        Callback function for odometry messages.
        """
        # Extract position from odometry
        position = msg.pose.pose.position
        self.x = position.y
        self.y = position.x
        
        # Extract orientation from odometry and compute yaw
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        # siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        # cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.yaw = yaw
        print("yaw: ", self.yaw)
        # Extract linear velocity from odometry and compute speed
        twist = msg.twist.twist.linear
        self.v = math.hypot(twist.x, twist.y)
        print("X: ", self.x)
        print("Y: ",self.y)
        print("Yaw: ",self.yaw)
        print("V: ",self.v)

def main():
    rospy.init_node('car_controller', anonymous=True)
        
    # Create an odometry handler object
    odom_handler = OdometryHandler()
    rospy.spin()

if __name__ == '__main__':
    main()
