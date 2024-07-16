#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Bool, String,Float32
from nav_msgs.msg import Odometry
import tf.transformations
import math
# from Final_lidar_distance import LidarSubscriber
from Stanley import main

class SituationHandler:
    commands_queue = []
    def __init__(self):
        rospy.init_node('situation_handler', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
        self.brakes_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
        self.steering_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
        self.sub = rospy.Subscriber('/situations', String, self.callback_state)
        rospy.Subscriber('/lidar_distance', Float32, self.callback_distance)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.sleep(0.5)  

        # Start simulation 
        # startSimulation = rospy.Publisher('/startSimulation', Bool, queue_size=10)
        # rospy.sleep(0.5)
        # startSimulation.publish(Bool(True))
        # rospy.sleep(0.5)

        self.current_distance = 0.0
        self.current_speed = 0.0

        # Change lane parameters
        self.current_position = None
        self.target_lane_center = 0.0  # Center of the lane
        self.lane_width_R = 4.0  # Width of the lane
        self.lane_width_L = 4.5  # Width of the lane
        self.max_steering_angle = 15.0  # Maximum steering angle in degrees
        self.change_in_progress = False
        self.orientation_q = 0.0
        self.current_orientation = 0.0
    
    def Stay_in_lane(self):
        self.brakes_pub.publish(0.0)
        self.cmd_vel_pub.publish(0.2)

    def Emergency_Stop(self):
        # self.sub.unregister()
        self.lock = 1
        self.cmd_vel_pub.publish(0.0)
        self.brakes_pub.publish(1.0)
        rospy.sleep(0.75)
        # try:
        #     # while len(self.commands_queue) and (self.commands_queue[0] == 'Emergency Stops' or self.commands_queue[0] == 'Gradually Stops'):
        #     #     self.commands_queue.pop(0)
        # except IndexError:
        #     rospy.logerr('Index Out Of Range')
        #     pass    
        self.lock = 0
        # self.sub = rospy.Subscriber('/situations', String, self.callback_state)
        
    def Gradually_Stop(self): # Slow Down
        # self.sub.unregister()
        self.lock = 1
        self.cmd_vel_pub.publish(0.0)
        self.brakes_pub.publish(0.2)
        self.lock = 0
        # self.sub = rospy.Subscriber('/situations', String, self.callback_state)

    def change_lane(self, direction):
        self.sub.unregister()
        if self.current_position is None:
            return

        self.target_lane_center = self.current_position.y
        self.steering_pub.publish(0.0)
        self.brakes_pub.publish(0.0)
        self.cmd_vel_pub.publish(0.1)

        if direction == 'left':
            self.target_lane_center -= self.lane_width_L
        elif direction == 'right':
            self.target_lane_center += self.lane_width_R

        self.change_in_progress = True

        while not rospy.is_shutdown() and self.change_in_progress:
            if self.current_position is None:
                rospy.loginfo("Waiting for position data...")
                continue
            
            error = abs(self.target_lane_center) - abs(self.current_position.y)

            if direction == 'left': 
                error = -error

            steering_angle = max(min(error * 5.0, self.max_steering_angle), -self.max_steering_angle)

            if abs(error) < 0.2:  # If close enough to the target
                self.change_in_progress = False
                rospy.loginfo("Lane change completed.")
                steering_angle = 0.0

            if direction == 'right':
                self.steering_pub.publish(Float64(-steering_angle))
            else:
                self.steering_pub.publish(Float64(steering_angle))
            self.cmd_vel_pub.publish(Float64(0.1))
            
        if direction == 'right':
            while self.current_orientation < 80.0:
                self.steering_pub.publish(Float64(steering_angle))
                steering_angle += 1
        else:
            while self.current_orientation > 105.5:
                self.steering_pub.publish(Float64(-steering_angle))
                steering_angle += 1

        for _ in range(4):
            if direction == 'right':
                pass
                self.steering_pub.publish(Float64(-steering_angle))
            else:
                self.steering_pub.publish(Float64(steering_angle))
        self.steering_pub.publish(0.0)
        # rospy.sleep(60)
        self.sub = rospy.Subscriber('/situations', String, self.callback_state)

    def handler(self):
        while not rospy.is_shutdown():
            while len(self.commands_queue) != 0:
                command = self.commands_queue.pop(0)
                # rospy.loginfo(f"Received command: {command}")
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
                    self.brakes_pub.publish(0.0)
            # rospy.loginfo('Out from loop')
            # self.brakes_pub.publish(0.0)

    def Adaptive_Cruise_Control(self):
        self.cmd_vel_pub.publish(0.0)
        self.brakes_pub.publish(0.0)

        # Initialize state variables if not already done
        if not hasattr(self, 'state'):
            self.state = {
                'K_p': 0.15,
                'K_d': 1.0,
                'K_i': 0.0003,
                'max_output': 1.0,
                'desired_distance': 1.5,
                'desired_speed': 3.0,
                'prev_setpoint': 0.0,
                'time_step': 0.1,
                'integral_setpoint': 0.0,
                'maintaining_distance': False
            }

        while not rospy.is_shutdown():
            delta_distance = self.current_distance - 2 * self.state['desired_distance'] - self.current_speed**2 / (2 * 2.11)

            # if the car ahead does not allow to get to cruise speed
            # use safe following distance as a measure until cruise speed is reached again
            if delta_distance < 0:
                self.state['maintaining_distance'] = True
            elif self.current_speed >= self.state['desired_speed'] :
                self.state['maintaining_distance'] = False

            if self.state['maintaining_distance']:
                # Override if we are too close to the car in front
                set_point = delta_distance
            else:
                # Maintain cruise speed if distance is not too close
                set_point = self.state['desired_speed'] - self.current_speed

            # Calculate control using PID
            control = (self.state['K_p'] * set_point +
                    self.state['K_d'] * (set_point - self.state['prev_setpoint']) +
                    self.state['K_i'] * self.state['integral_setpoint'])

            # Constrain control to the range -1.0 to 1.0
            control = max(min(control, self.state['max_output']), -self.state['max_output'])

            rospy.loginfo("Distance: %.2f, Set Point: %.2f, Control: %.2f" % (self.current_distance, set_point, control))

            # Adjust speed and brakes based on control
            if control >= 0:
                self.cmd_vel_pub.publish(control)
                self.brakes_pub.publish(0.0)
            else:
                self.cmd_vel_pub.publish(0.0)
                self.brakes_pub.publish(-control)

            # Update state variables
            self.state['prev_setpoint'] = set_point
            self.state['integral_setpoint'] += set_point

            # Exit loop if a new command is received
            if (len(self.commands_queue) != 0) and (self.commands_queue[0] != "Adaptive Cruise Control"):
                break

            rospy.sleep(self.state['time_step'])


    def callback_state(self, data):
        # if data.data != 'All Good':
        self.commands_queue.append(data.data)
        # if len(self.commands_queue):
        #     rospy.loginfo(f'---- Queue: {self.commands_queue} ----')

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
        node = SituationHandler()
        node.handler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass