#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class OdometryImuNode:
    def __init__(self):
        rospy.init_node('odometry_imu_node', anonymous=True)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('/Imu', Imu, queue_size=10)
        
        rospy.Subscriber('/odometry/data', Odometry, self.odom_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        
    def odom_callback(self, data):
        self.odom_pub.publish(data)

    def imu_callback(self, data):
        self.imu_pub.publish(data)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = OdometryImuNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
