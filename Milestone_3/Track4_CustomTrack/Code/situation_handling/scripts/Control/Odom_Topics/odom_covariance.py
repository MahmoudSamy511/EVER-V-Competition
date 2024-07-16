#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

odom_pub = None

posX = 0.1
posY = 0.1
posZ = 99999.0
rotX = 99999.0
rotY = 99999.0
rotZ = 0.1

def odomCallback(msg):
    global posX, posY, posZ, rotX, rotY, rotZ

    odom = Odometry()
    odom.child_frame_id = msg.child_frame_id
    odom.header = msg.header
    odom.header.frame_id = "base_footprint"  # the tracked robot frame
    odom.pose = msg.pose
    odom.twist = msg.twist

    odom.pose.covariance[0] = posX
    odom.pose.covariance[7] = posY
    odom.pose.covariance[14] = posZ
    odom.pose.covariance[21] = rotX
    odom.pose.covariance[28] = rotY
    odom.pose.covariance[35] = rotZ
    odom.twist.covariance = odom.pose.covariance

    odom_pub.publish(odom)

def main():
    global posX, posY, posZ, rotX, rotY, rotZ, odom_pub

    rospy.init_node("odom_covariance_node")
    n = rospy.get_namespace()

    posX = rospy.get_param(n + "posX", 0.1)
    posY = rospy.get_param(n + "posY", 0.1)
    posZ = rospy.get_param(n + "posZ", 99999.0)
    rotX = rospy.get_param(n + "rotX", 99999.0)
    rotY = rospy.get_param(n + "rotY", 99999.0)
    rotZ = rospy.get_param(n + "rotZ", 0.1)

    sub = rospy.Subscriber("odom", Odometry, odomCallback)
    odom_pub = rospy.Publisher("odom_cov", Odometry, queue_size=1)

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
