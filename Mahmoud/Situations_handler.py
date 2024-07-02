#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64, Int64, Bool
from nav_msgs.msg import Odometry
import threading

class SituationHandler:
    commands_queue = []

    def __init__(self):
        self.lock = threading.Lock()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
        self.brakes_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
        self.steering_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
        
        self.sub = rospy.Subscriber('/situations', String, self.callback_state)
        rospy.Subscriber('/distance', Int64, self.callback_distance)
        rospy.Subscriber('/odom', Odometry, self.callback_speed)

        rospy.sleep(1)  

        self.desired_distance = 3
        self.current_distance = 0
        self.Kp = 0.5  # Proportional gain
        self.Ki = 0.1  # Integral gain
        self.Kd = 0.05 # Derivative gain
        self.integral = 0
        self.previous_error = 0
        self.dt = 0.1  # Time step

    def Emergency_Stop(self): # Hand Brake
        self.sub.unregister()

        print("Emergency Stop mode")
        self.brakes_pub.publish(1.0)

        self.sub = rospy.Subscriber('/situations', String, self.callback_state)
        
    def Gradually_Stop(self): # Done
        self.sub.unregister()

        print("Gradually Stop mode")
        self.brakes_pub.publish(0.5)
        self.cmd_vel_pub.publish(0.1)

        self.sub = rospy.Subscriber('/situations', String, self.callback_state)

    def Adaptive_Cruise_Control(self):      # Need Testing on the car
        self.sub.unregister()

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

        self.sub = rospy.Subscriber('/situations', String, self.callback_state)
        

    def Lane_Change_to_right(self):            # Finished 
        self.sub.unregister()
        rospy.sleep(1.1)  # Wait for publishers to register.

        # Send gas pedal command and start turning
        self.steering_pub.publish(Float64(-9.0))
        rospy.sleep(1.5)
        # Straight 
        self.steering_pub.publish(Float64(0.0))
        rospy.sleep(0.175)
        # Justify lane
        self.steering_pub.publish(Float64(9.0))
        rospy.sleep(1.5)
        # Restore State
        self.brakes_pub.publish(Float64(0.0))
        self.cmd_vel_pub.publish(Float64(0.2))
        self.steering_pub.publish(Float64(0.0))

        self.sub = rospy.Subscriber('/situations', String, self.callback_state)

    def Lane_Change_to_left(self): # Finished
        self.sub.unregister()

        rospy.sleep(0.5)  # Wait for publishers to register.
        # Send gas pedal command and start turning
        self.steering_pub.publish(Float64(9.0))
        rospy.sleep(1.5)
        # Straight 
        self.steering_pub.publish(Float64(0.0))
        rospy.sleep(0.175)
        # Justify lane
        self.steering_pub.publish(Float64(-9.0))
        rospy.sleep(1.3)
        # Restore State
        self.brakes_pub.publish(Float64(0.0))
        self.cmd_vel_pub.publish(Float64(0.2))
        self.steering_pub.publish(Float64(0.0))

        self.sub = rospy.Subscriber('/situations', String, self.callback_state)

    def Stay_in_lane(self): # Done
        print("Stay in lane mode")
        self.brakes_pub.publish(0.0)
        self.cmd_vel_pub.publish(0.1)
        rospy.sleep(3)

    def callback_state(self, data):
        self.commands_queue.append(data.data)

    def handler(self):
        while not rospy.is_shutdown():
            if self.commands_queue:
                command = self.commands_queue.pop(0)
                print("Processing command:", command)
                if command == "Emergency Stops":
                    self.Emergency_Stop()
                elif command == "Adaptive Cruise Control":
                    self.Adaptive_Cruise_Control()
                elif command == "Lane Change to the right":
                    self.Lane_Change_to_right()
                elif command == "Lane Change to the left":
                    self.Lane_Change_to_left()
                elif command == "Gradually Stops":
                    self.Gradually_Stop()
                else:
                    self.Stay_in_lane()
            else:
                rospy.sleep(0.1)

    def callback_distance(self, data):
        self.current_distance = data.data

    def callback_speed(self, data):
        self.current_speed = data.twist.twist.linear.x

if __name__ == '__main__':
    try:
        rospy.init_node('situation_handler', anonymous=True)
        # Start simulation 
        startSimulation = rospy.Publisher('/startSimulation', Bool, queue_size=10)
        startSimulation.publish(Bool(True))
        rospy.sleep(0.5)
        
        node = SituationHandler()
        node.handler()

    except rospy.ROSInterruptException:
        pass
