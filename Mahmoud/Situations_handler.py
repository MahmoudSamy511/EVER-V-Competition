#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Int64
from nav_msgs.msg import Odometry
class SituationHandler:
    def __init__(self):
        rospy.init_node('situation_handler', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
        self.brakes_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
        self.steering_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
        
        rospy.Subscriber('situations', String, self.callback_state)
        rospy.Subscriber('distance', Int64, self.callback_distance)
        rospy.Subscriber('odom', Odometry, self.callback_speed)

        rospy.sleep(1)  


        self.current_speed = 0.0

        self.desired_distance = 3
        self.current_distance = 0
        self.Kp = 0.5  # Proportional gain
        self.Ki = 0.1  # Integral gain
        self.Kd = 0.05 # Derivative gain
        self.integral = 0
        self.previous_error = 0
        self.dt = 0.1  # Time step
        
        # rospy.spin()

    def Emergency_Stop(self): # Hand Brake
        print("Emergency Stop mode")
        self.brakes_pub.publish(1.0)

    def Gradually_Stop(self): # Done
        print("Gradually Stop mode")
        self.brakes_pub.publish(0.5)
        self.cmd_vel_pub.publish(0.1)


    def Adaptive_Cruise_Control(self):      # Need Testing on the car
        print("Adaptive Cruise Control mode")
        error = self.desired_distance - self.current_distance
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        
        # Adjust speed and brakes
        if output > 0:
            self.cmd_vel_pub.publish(min(output, 1.0))
            self.brakes_pub.publish(0.0)
        else:
            self.cmd_vel_pub.publish(0.0)
            self.brakes_pub.publish(min(-output, 1.0))

    def Lane_Change_to_right(self):            # Not finished 
        # self.cmd_vel_pub.publish(0.6)
        self.current_speed = 0.6
        if self.current_speed > 0.5:
            print("Lane Changing to the right at high speed.")
            self.steering_pub.publish(-30)  # Example value to steer right
            self.cmd_vel_pub.publish(self.current_speed)
            rospy.sleep(3)
            self.steering_pub.publish(30)  # Example value to steer right
            self.cmd_vel_pub.publish(self.current_speed)
            rospy.sleep(2)
            self.steering_pub.publish(0)  # Example value to steer right
            self.cmd_vel_pub.publish(self.current_speed)
        else:
            print("Lane Changing to the right at low speed.")
            self.steering_pub.publish(-0.3*30)  # Example value to steer right
            self.cmd_vel_pub.publish(self.current_speed * 0.8)

    def Lane_Change_to_left(self):        # Not finished 
        if self.current_speed > 0.5:
            print("Lane Changing to the left at high speed.")
            self.steering_pub.publish(0.5)  # Example value to steer left
            self.cmd_vel_pub.publish(self.current_speed)
        else:
            print("Lane Changing to the left at low speed.")
            self.steering_pub.publish(0.3)  # Example value to steer left
            self.cmd_vel_pub.publish(self.current_speed * 0.8)

    def Stay_in_lane(self): # Done
        print("Stay in lane mode")
        self.brakes_pub.publish(0.0)
        self.cmd_vel_pub.publish(0.1)
        rospy.sleep(3)


    def callback_state(self, data):
        if data.data == "Emergency Stops":
            self.Emergency_Stop()
        elif data.data == "Adaptive Cruise Control":
            self.Adaptive_Cruise_Control()
        elif data.data == "Lane Change to the right":
            self.Lane_Change_to_right()
        elif data.data == "Lane Change to the left":
            self.Lane_Change_to_left()
        elif data.data == "Gradually Stops":
            self.Gradually_Stop()
        else:
            self.Stay_in_lane()

    def callback_distance(self, data):
        self.current_distance = data.data

    def callback_speed(self, data):
        self.current_speed = data.twist.twist.linear.x

if __name__ == '__main__':
    try:
        node = SituationHandler()
        # node.Stay_in_lane()
        # node.Gradually_Stop()
        node.Lane_Change_to_right()

    except rospy.ROSInterruptException:
        pass