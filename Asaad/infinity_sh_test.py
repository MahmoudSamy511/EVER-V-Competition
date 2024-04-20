#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import time

def move_back():
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
    gas_pedal_msg.data = -0.1    
    cmd_vel_pub.publish(gas_pedal_msg)
    
    steering_msg = Float64()
    steering_msg.data = 0.0  # Steering angle for turning (adjust as needed)
    steering_pub.publish(steering_msg)
    
    rospy.sleep(6)  # Sleep for 2 seconds
    
    # Stop turning and apply brakes
    steering_pub.publish(Float64())  # Stop turning
    brakes_msg.data = 0.5
    cmd_vel_pub.publish(Float64())  # Stop gas pedal
    brakes_pub.publish(brakes_msg)

def steer():
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
    gas_pedal_msg.data = 0.25    
    cmd_vel_pub.publish(gas_pedal_msg)
    
    steering_msg = Float64()
    steering_msg.data = 10.0  # Steering angle for turning (adjust as needed)
    steering_pub.publish(steering_msg)
    
    rospy.sleep(9.7)  # Sleep for 2 seconds
    
    # Stop turning and apply brakes
    steering_pub.publish(Float64())  # Stop turning
    brakes_msg.data = 0.5
    cmd_vel_pub.publish(Float64())  # Stop gas pedal
    brakes_pub.publish(brakes_msg)

def rev_steer():
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
    gas_pedal_msg.data = 0.25    
    cmd_vel_pub.publish(gas_pedal_msg)
    
    steering_msg = Float64()
    steering_msg.data = -10.0  # Steering angle for turning (adjust as needed)
    steering_pub.publish(steering_msg)
    
    rospy.sleep(9)  # Sleep for 2 seconds
    
    # Stop turning and apply brakes
    steering_pub.publish(Float64())  # Stop turning
    brakes_msg.data = 0.65
    cmd_vel_pub.publish(Float64())  # Stop gas pedal
    brakes_pub.publish(brakes_msg)



if __name__ == '__main__':
    try:
        # move_back()
        steer()     	# First Circle
        rev_steer() 	# Second Circle	
    except rospy.ROSInterruptException:
        pass