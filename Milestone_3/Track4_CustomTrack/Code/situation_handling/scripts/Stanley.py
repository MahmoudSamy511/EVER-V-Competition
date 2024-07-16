#!/usr/bin/env python3

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
from std_msgs.msg import String

# Import your local modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "MotionPlanning/")
import Control.draw as draw
import CurvesGenerator.cubic_spline as cs

start_time = time.time()
counter = 0
reach_flag = False
path_x = []
path_y = []
path_yaw = []
last_start_index, last_end_index = None, None
controller = 0

# Constants class
class C:
    # PID config
    Kp = 1.2

    # System config
    k = 3.5
    dt = 0.1
    dref = 4

    # vehicle config
    RF = 3.3    # [m] distance from rear to vehicle front end of vehicle
    RB = 0.8    # [m] distance from rear to vehicle back end of vehicle
    W = 2.4     # [m] width of vehicle
    WD = 0.7 * W  # [m] distance between left-right wheels
    WB = 2.5      # [m] Wheel base
    TR = 0.44     # [m] Tyre radius
    TW = 0.7      # [m] Tyre width
    MAX_STEER = 0.65


class OdometryHandler:
    """
    Class to handle odometry data.
    """
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.path_x = []
        self.path_y = []
        self.path_yaw = []
        # Initialize the subscriber to receive odometry messages
        rospy.Subscriber('/odom', Odometry, self.log_callback_odom)
        rospy.Subscriber('/cruising_path', String, self.path_callback)

    def log_callback_odom(self, msg):
        """
        Callback function for odometry messages.
        """
        # Extract position from odometry
        position = msg.pose.pose.position
        self.x = position.y
        self.y = -position.x
        
        # Extract orientation from odometry and compute yaw
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.yaw = yaw
        # print("yaw: ", self.yaw)
        # Extract linear velocity from odometry and compute speed
        twist = msg.twist.twist.linear
        self.v = math.hypot(twist.x, twist.y)
        
    def path_callback(self, msg):
        global start_time
        global counter
        if time.time() - start_time > 1 or counter == 0:
            counter = 1
            start_time = time.time()
            data = msg.data.split(';')
            global path_x
            global path_y
            global path_yaw

            for item in data:
                if item.startswith("x:"):
                    path_x = list(map(float, item[2:].split(',')))
                elif item.startswith("y:"):
                    path_y = list(map(float, item[2:].split(',')))
                elif item.startswith("yaw:"):
                    path_yaw = list(map(float, item[4:].split(',')))
        


class Node:
    """
    Node class representing a vehicle.
    """
    def __init__(self, odom_handler, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

        # Store a reference to the odometry handler object
        self.odom_handler = odom_handler
        
        # Initialize ROS node
        rospy.init_node('car_controller', anonymous=True)
        
        # ROS publishers for steering angle and cmd_vel (gas pedal position)
        self.steering_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
        self.brakes = rospy.Publisher('/brakes', Float64, queue_size=10)
        # Initialize the reference path flag
        self.reference_set = False

    def update(self, a, delta):
        """
        Update function that publishes the steering angle and gas pedal position.
        """
        # Convert delta from radians to degrees
        steering_angle_deg = delta * 180 / math.pi
        # Publish the steering angle
        self.steering_pub.publish(Float64(steering_angle_deg))

        # Clamp acceleration between 0 and 1 for the gas pedal
        gas_pedal = np.clip(a, 0, 1)
        # Publish the gas pedal position
        self.cmd_vel_pub.publish(0.1)
    
    def find_nearest_index(self, ref_x, ref_y, target_x, target_y):
        # Calculate distances from target point to all points in ref_x and ref_y
        distances = np.sqrt((np.array(ref_x) - target_x) ** 2 + (np.array(ref_y) - target_y) ** 2)
        # Find the index of the nearest point
        return np.argmin(distances)

    def run(self, ref_path, target_speed):
        global controller
        global last_start_index, last_end_index
        counter = 0
        """
        Run the control loop for the node.
        """
        # Initialize variables
        t = 0.0
        max_time = 100000
        xrec, yrec, yawrec = [], [], []
        c = 0
        
    
        # Control loop
        while t < max_time:
            # rospy.loginfo(f'controller {controller}')
            # rospy.loginfo()
            if controller == 0:
                # Get the latest odometry data from the odometry handler
                self.x = self.odom_handler.x
                self.y = self.odom_handler.y
                self.yaw = self.odom_handler.yaw
                self.v = self.odom_handler.v
                global path_x, path_y, path_yaw
                
                # Calculate feedback control
                delta, target_index = front_wheel_feedback_control(self, ref_path)
                
                # Calculate the distance to the end of the path
                dist = math.hypot(self.x - ref_path.cx[-1], self.y - ref_path.cy[-1])
                
                # Calculate acceleration using PID control
                ai = pid_control(target_speed, self.v, dist)
                
                # Update the node's control inputs
                self.update(ai, delta)
                
                # Update time
                t += C.dt
            
            # Save the trajectory
                xrec.append(self.x)
                yrec.append(self.y)
                yawrec.append(self.yaw)
                
            # Plot the trajectory
            # plt.cla()/
            # plt.plot(ref_path.cx, ref_path.cy, color='gray', linewidth=2.0)
            # plt.plot(xrec, yrec, linewidth=2.0, color='darkviolet')
            # plt.plot(ref_path.cx[target_index], ref_path.cy[target_index], color='green')
            # draw.draw_car(self.x, self.y, self.yaw, delta, C)
            # plt.axis("equal")
            # plt.title(f"Front Wheel Feedback Control: v={self.v * 3.6:.2f} km/h")
            # plt.pause(0.001)

                plt.show()
            else:
                pass

class Trajectory:
    def __init__(self, cx, cy, cyaw):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ind_old = 0

    def calc_theta_e_and_ef(self, node):
        """
        Calculate theta_e (heading error) and ef (lateral error) given the node's current state.
        """
        # Calculate front axle coordinates
        fx = node.x + C.WB * math.cos(node.yaw)
        fy = node.y + C.WB * math.sin(node.yaw)

        # Calculate the differences between the front axle and the reference path
        dx = [fx - x for x in self.cx]
        dy = [fy - y for y in self.cy]

        # Find the closest point on the reference path
        target_index = np.argmin(np.hypot(dx, dy))
        target_index = max(self.ind_old, target_index)
        self.ind_old = max(self.ind_old, target_index)

        # Calculate lateral error
        front_axle_vec_rot_90 = np.array([[math.cos(node.yaw - math.pi / 2.0)],
                                          [math.sin(node.yaw - math.pi / 2.0)]])
        vec_target_2_front = np.array([[dx[target_index]],
                                       [dy[target_index]]])
        ef = np.dot(vec_target_2_front.T, front_axle_vec_rot_90)

        # Calculate heading error
        theta = node.yaw
        theta_p = self.cyaw[target_index]
        theta_e = pi_2_pi(theta_p - theta)

        return theta_e, ef, target_index

def front_wheel_feedback_control(node, ref_path):
    """
    Front wheel feedback control algorithm.
    """

    # Calculate heading error and lateral error
    theta_e, ef, target_index = ref_path.calc_theta_e_and_ef(node)
    
    # Calculate steering angle
    delta = theta_e + math.atan2(C.k * ef, node.v)
    # print("Delta: ", delta)
    
    return delta, target_index

def pi_2_pi(angle):
    """
    Normalize an angle to the range [-pi, pi].
    """
    if angle > math.pi:
        return angle - 2.0 * math.pi
    if angle < -math.pi:
        return angle + 2.0 * math.pi

    return angle    

def pid_control(target_v, v, dist):
    """
    PID control algorithm for speed control.
    """
    a = 0.3 * (target_v - v)

    if dist < 10.0:
        if v > 3.0:
            a = -2.5
        elif v < -2.0:
            a = -1.0

    return a

latest_val = 0

def callback_state(msg):
    global controller
    print(msg.data, controller)
    # global latest_val
    rospy.loginfo(f'data: {msg.data}')
    if msg.data == 'All Good' and controller == 1:
        rospy.sleep(40)
        controller = 0
        # Node.brakes.publish(0.0)
        rospy.loginfo('ONE')
    elif msg.data == 'All Good' and controller == 2:
        controller = 0
        rospy.loginfo('TWO')
    elif msg.data == 'Emergency Stops':
        controller = 2
    elif msg.data == "Lane Change to the left" or msg.data == "Lane Change to the right":
        controller = 1

def main():
    rospy.Subscriber('/situations', String, callback_state)

    # Load reference path from CSV file
    innovation = 'innovation.csv'

    with open(innovation, newline='') as f:
        rows = list(reader(f, delimiter=','))
    
    # Create an odometry handler object
    odom_handler = OdometryHandler()

    # Parse the path from the CSV file
    ax, ay = [[float(i) for i in row] for row in zip(*rows[1:])]
    ax[0] = odom_handler.x
    ay[0] = odom_handler.y
    
    # Generate the reference path
    cx, cy, cyaw, _, _ = cs.calc_spline_course(ax, ay, ds=C.dt)
    cyaw[0] = odom_handler.yaw
    ref_path = Trajectory(cx, cy, cyaw)

    # Initialize the node with the odometry handler
    node = Node(odom_handler, x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)
    
    # Run the control loop
    node.run(ref_path, target_speed= 25.0 / 3.6)  # Target speed in m/s

if __name__ == '__main__':
    main()