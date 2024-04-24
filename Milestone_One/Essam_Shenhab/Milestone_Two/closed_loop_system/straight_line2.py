#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import time

def straight_line_callback(odom_data):
   
    if odom_data.pose.pose.position.y < 67.5:  
        gas_pedal_msg = Float64()
        gas_pedal_msg.data = 0.2
        cmd_vel_pub.publish(gas_pedal_msg)
    else:
        steering_pub.publish(Float64())  # Stop turning
        brakes_msg = Float64()
        brakes_msg.data = 0.65
        cmd_vel_pub.publish(Float64())  # Stop gas pedal
        brakes_pub.publish(brakes_msg)



if __name__ == '__main__':
    try:
        rospy.init_node('simple_car_controller', anonymous=True)

        # Define ROS publishers
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
        brakes_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
        steering_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)

        rospy.Subscriber('/odom', Odometry, straight_line_callback)

        rospy.spin() 

    except rospy.ROSInterruptException:
        pass
