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

# Import your local modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../MotionPlanning/")
import Control.draw as draw
import CurvesGenerator.cubic_spline as cs

reach_flag = False

# Constants class
class C:
    # PID config
    Kp = 1.2

    # System config
    k = 0.5
    dt = 0.1
    dref = 1.2

    # vehicle config
    RF = 3.3  # [m] distance from rear to vehicle front end of vehicle
    RB = 0.8  # [m] distance from rear to vehicle back end of vehicle
    W = 2.4  # [m] width of vehicle
    WD = 0.7 * W  # [m] distance between left-right wheels
    WB = 2.5  # [m] Wheel base
    TR = 0.44  # [m] Tyre radius
    TW = 0.7  # [m] Tyre width
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
        
        # Initialize the subscriber to receive odometry messages
        rospy.Subscriber('/odom', Odometry, self.log_callback_odom)

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
        # siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        # cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.yaw = yaw
        # print("yaw: ", self.yaw)
        # Extract linear velocity from odometry and compute speed
        twist = msg.twist.twist.linear
        self.v = math.hypot(twist.x, twist.y)


class Node:
    """
    Node class representing a vehicle.
    """
    def __init__(self, odom_handler, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.counter = 0
        
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
        self.cmd_vel_pub.publish(gas_pedal + 0.2)

    def run(self, ref_path, target_speed):
        """
        Run the control loop for the node.
        """
        # Initialize variables
        t = 0.0
        max_time = 1000
        xrec, yrec, yawrec = [], [], []
        c = 0
        
        # Control loop
        while t < max_time:
            # Get the latest odometry data from the odometry handler
            self.x = self.odom_handler.x
            self.y = self.odom_handler.y
            self.yaw = self.odom_handler.yaw
            self.v = self.odom_handler.v
            
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
            
            # Check if the vehicle is close to the end of the path
            if dist <= C.dref:
                self.brakes.publish(Float64(0.05))
                self.cmd_vel_pub.publish(Float64(0))
                break
            
            # Save the trajectory
            xrec.append(self.x)
            yrec.append(self.y)
            yawrec.append(self.yaw)
            
            # Plot the trajectory
            plt.cla()
            plt.plot(ref_path.cx, ref_path.cy, color='gray', linewidth=2.0)
            plt.plot(xrec, yrec, linewidth=2.0, color='darkviolet')
            plt.plot(ref_path.cx[target_index], ref_path.cy[target_index], color='green')
            draw.draw_car(self.x, self.y, self.yaw, delta, C)
            plt.axis("equal")
            plt.title(f"Front Wheel Feedback Control: v={self.v * 3.6:.2f} km/h")
            plt.pause(0.001)

        plt.show()

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

def main():
    # Load reference path from CSV file
    with open('bigger_infinity_shape_path.csv', newline='') as f:
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
    node.run(ref_path, target_speed=25.0 / 3.6)  # Target speed in m/s

if __name__ == '__main__':
    main()
