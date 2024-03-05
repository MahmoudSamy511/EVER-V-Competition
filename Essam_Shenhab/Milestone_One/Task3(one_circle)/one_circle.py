#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import time

def one_circle():
    rospy.init_node('simple_car_controller', anonymous=True)
    
    # Define ROS publishers
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
    brakes_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
    steering_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
    
    rospy.sleep(1)  # Wait for publishers to register
    
    # Send gas pedal command and start turning
    brakes_msg = Float64()
    brakes_msg.data = 0.0
    brakes_pub.publish(brakes_msg)

    gas_pedal_msg = Float64()
    gas_pedal_msg.data = 0.2
    cmd_vel_pub.publish(gas_pedal_msg)
    
    steering_msg = Float64()
    steering_msg.data = 17.9597315  # Steering angle for turning (adjust as needed)
    steering_pub.publish(steering_msg)
    
    rospy.sleep(9.1)  # Sleep for 2 seconds
    
    # Stop turning and apply brakes
    brakes_msg.data = 0.5211
    cmd_vel_pub.publish(Float64())  # Stop gas pedal
    brakes_pub.publish(brakes_msg)

    rospy.sleep(7)
    

if __name__ == '__main__':
    try:
        one_circle()
    except rospy.ROSInterruptException:
        pass