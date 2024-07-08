#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64, Bool
from nav_msgs.msg import Odometry
import threading

class SituationHandler:
    commands_queue = []

    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
        self.brakes_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
        self.steering_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
        
        self.sub = rospy.Subscriber('/situations', String, self.callback_state)
        rospy.Subscriber('/distance_front', Float64, self.callback_distance)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.sleep(1)  

        self.desired_distance = 3
        self.current_distance = 0.0
        self.Kp = 0.5  # Proportional gain
        self.Ki = 0.1  # Integral gain
        self.Kd = 0.05 # Derivative gain
        self.integral = 0
        self.previous_error = 0
        self.dt = 0.1  # Time step

        # Change lane parameters
        self.current_position = None
        self.target_lane_center = 0.0  # Center of the lane
        self.lane_width = 5.0
        self.max_steering_angle = 30.0  # Maximum steering angle in degrees
        self.change_in_progress = False
        self.orientation_q = 0.0
        self.current_orientation = 0.0

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

    def Adaptive_Cruise_Control(self): # Need Testing on the car
        while True:
            # error = self.desired_distance - self.current_distance
            # self.integral += error * self.dt
            # derivative = (error - self.previous_error) / self.dt
            # output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            # self.previous_error = error
            # # Adjust speed and brakes
            # if output > 0:
            #     self.cmd_vel_pub.publish(min(output, 1.0))
            #     self.brakes_pub.publish(0.0)
            #     # rospy.sleep(1)
            # else:
            #     self.cmd_vel_pub.publish(0.0)
            #     self.brakes_pub.publish(min(-output, 1.0))
            #     # rospy.sleep(1)

            if self.current_distance <  self.desired_distance:
                self.brakes_pub.publish(1.0)
                self.cmd_vel_pub.publish(0.0)
            else:
                self.cmd_vel_pub.publish(0.5)
                self.brakes_pub.publish(0.0)

            if len(self.commands_queue) != 0 and self.commands_queue.pop(0) != "Adaptive Cruise Control":
                break
        

        def change_lane(self, direction):
            rospy.loginfo(f'direction is {direction}')
            self.target_lane_center = 0.0
            self.steering_angle_pub.publish(0.0)
            if self.current_position is None:
                return

            if direction == 'left':
                self.target_lane_center -= self.lane_width
            elif direction == 'right':
                self.target_lane_center += self.lane_width

            self.change_in_progress = True

            while not rospy.is_shutdown() and self.change_in_progress:
                if self.current_position is None:
                    rospy.loginfo("Waiting for position data...")
                    continue

                error = abs(self.target_lane_center) - abs(self.current_position.y)

                steering_angle = max(min(error * 5.0, self.max_steering_angle), -self.max_steering_angle)
                rospy.loginfo("Current position: y = %.2f, Target position: y = %.2f, Steering angle: %.2f" %
                            (self.current_position.y, self.target_lane_center, steering_angle))

                if abs(error) < 0.2:  # If close enough to the target
                    self.change_in_progress = False
                    rospy.loginfo("Lane change completed.")
                    steering_angle = 0.0

                # Publish commands
                if direction == 'right':
                    self.steering_angle_pub.publish(Float64(-1 * steering_angle))
                else:
                    self.steering_angle_pub.publish(Float64(steering_angle))
                
                self.cmd_vel_pub.publish(Float64(0.1))
                
            
            if direction == 'right':
                while self.current_orientation < 78.43:
                    self.steering_angle_pub.publish(Float64(steering_angle))
                    steering_angle += 1
            else:
                while self.current_orientation > 102.0:
                    self.steering_angle_pub.publish(Float64(-steering_angle))
                    steering_angle += 1


            for _ in range(4):
                if direction == 'right':
                    self.steering_angle_pub.publish(Float64(-1 * steering_angle))
                else:
                    self.steering_angle_pub.publish(Float64(steering_angle))
            
            self.steering_angle_pub.publish(0.0)

    # def Lane_Change_to_right(self):            # Finished But NOT USED
    #     self.sub.unregister()
    #     rospy.sleep(1.1)  # Wait for publishers to register.

    #     # Send gas pedal command and start turning
    #     self.steering_pub.publish(Float64(-9.0))
    #     rospy.sleep(1.5)
    #     # Straight 
    #     self.steering_pub.publish(Float64(0.0))
    #     rospy.sleep(0.175)
    #     # Justify lane
    #     self.steering_pub.publish(Float64(9.0))
    #     rospy.sleep(1.5)
    #     # Restore State
    #     self.brakes_pub.publish(Float64(0.0))
    #     self.cmd_vel_pub.publish(Float64(0.2))
    #     self.steering_pub.publish(Float64(0.0))

    #     self.sub = rospy.Subscriber('/situations', String, self.callback_state)

    # def Lane_Change_to_left(self): # Finished
    #     self.sub.unregister()

    #     rospy.sleep(0.5)  # Wait for publishers to register.
    #     # Send gas pedal command and start turning
    #     self.steering_pub.publish(Float64(9.0))
    #     rospy.sleep(1.5)
    #     # Straight 
    #     self.steering_pub.publish(Float64(0.0))
    #     rospy.sleep(0.175)
    #     # Justify lane
    #     self.steering_pub.publish(Float64(-9.0))
    #     rospy.sleep(1.3)
    #     # Restore State
    #     self.brakes_pub.publish(Float64(0.0))
    #     self.cmd_vel_pub.publish(Float64(0.2))
    #     self.steering_pub.publish(Float64(0.0))

    #     self.sub = rospy.Subscriber('/situations', String, self.callback_state)

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
                    self.change_lane('right')
                elif command == "Lane Change to the left":
                    self.change_lane('left')
                elif command == "Gradually Stops":
                    self.Gradually_Stop()
                else:
                    self.Stay_in_lane()
            else:
                rospy.sleep(0.1)

    def callback_distance(self, data):
        self.current_distance = data.data

    def odom_callback(self, data):
        self.current_speed = data.twist.twist.linear.x
        
        self.current_position = data.pose.pose.position
        self.orientation_q = data.pose.pose.orientation

        orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.current_orientation = math.degrees(yaw)

if __name__ == '__main__':
    try:
        rospy.init_node('situation_handler', anonymous=True)
        
        # Start simulation 
        startSimulation = rospy.Publisher('/startSimulation', Bool, queue_size=10)
        rospy.sleep(0.5)
        startSimulation.publish(Bool(True))
        rospy.sleep(0.5)

        node = SituationHandler()
        node.handler()

    except rospy.ROSInterruptException:
        pass
