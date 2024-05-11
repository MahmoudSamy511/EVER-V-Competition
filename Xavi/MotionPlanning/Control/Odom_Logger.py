#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from std_msgs.msg import Header
import numpy as np
from pykalman import KalmanFilter
from tf.transformations import euler_from_quaternion
import csv
import os

class OdometryHandler:
    """
    Class to handle odometry data, add Gaussian noise, filter it using Kalman filters, and log the data.
    """
    def __init__(self):
        # Initialize state variables
        self.state = {
            "position_x": 0.0,
            "position_y": 0.0,
            "orientation_x": 0.0,
            "orientation_y": 0.0,
            "orientation_z": 0.0,
            "orientation_w": 0.0,
            "linear_x": 0.0,
            "linear_y": 0.0,
            "linear_z": 0.0
        }

        # Initialize subscriber for odometry messages
        rospy.Subscriber('/odom', Odometry, self.log_callback_odom)
        
        # Initialize publisher for filtered odometry data
        self.filtered_pub = rospy.Publisher('/filtered_odom', Odometry, queue_size=10)

        # Initialize Kalman filters for position, orientation, and linear velocity
        self.init_kalman_filters()

        # Define CSV file paths
        self.csv_before_path = 'before_anything.csv'
        self.csv_noise_path = 'after_noise_adding.csv'
        self.csv_filtered_path = 'after_filtering.csv'

        # Write headers if files are empty
        self.write_csv_headers()


    def init_kalman_filters(self):
        # Initialize Kalman filters for position, orientation, and linear velocity
        self.position_filter = KalmanFilter(
            transition_matrices=np.array([[1, 0, 1, 0],
                                          [0, 1, 0, 1],
                                          [0, 0, 1, 0],
                                          [0, 0, 0, 1]]),
            observation_matrices=np.array([[1, 0, 0, 0],
                                           [0, 1, 0, 0]]),
            transition_covariance=np.eye(4) * 0.01,
            observation_covariance=np.eye(2) * 0.1,
            initial_state_mean=np.array([self.state["position_x"], self.state["position_y"], 0, 0]),
            initial_state_covariance=np.eye(4) * 1.0
        )

        self.orientation_filter = KalmanFilter(
            transition_matrices=np.eye(4),
            observation_matrices=np.eye(4),
            transition_covariance=np.eye(4) * 0.01,
            observation_covariance=np.eye(4) * 0.1,
            initial_state_mean=np.array([self.state["orientation_x"], self.state["orientation_y"], self.state["orientation_z"], self.state["orientation_w"]]),
            initial_state_covariance=np.eye(4) * 1.0
        )

        self.velocity_filter = KalmanFilter(
            transition_matrices=np.eye(3),
            observation_matrices=np.eye(3),
            transition_covariance=np.eye(3) * 0.01,
            observation_covariance=np.eye(3) * 0.1,
            initial_state_mean=np.array([self.state["linear_x"], self.state["linear_y"], self.state["linear_z"]]),
            initial_state_covariance=np.eye(3) * 1.0
        )

    def write_csv_headers(self):
        # Write headers to CSV files if they are empty
        if not os.path.exists(self.csv_before_path):
            with open(self.csv_before_path, 'w', newline='') as f:
                csv.writer(f).writerow(['timestamp', 'x', 'y', 'yaw'])
        if not os.path.exists(self.csv_noise_path):
            with open(self.csv_noise_path, 'w', newline='') as f:
                csv.writer(f).writerow(['timestamp', 'x', 'y'])
        if not os.path.exists(self.csv_filtered_path):
            with open(self.csv_filtered_path, 'w', newline='') as f:
                csv.writer(f).writerow(['timestamp', 'x', 'y', 'yaw'])

    def filter_position_data(self, position_data, timestamp):
        """
        Filter position data using Kalman filter and log data.
        """
        # Filter position data using Kalman filter
        state_mean, state_covariance = self.position_filter.filter_update(
            self.position_filter.initial_state_mean,
            self.position_filter.initial_state_covariance,
            position_data  # Pass noisy data directly
        )

        # Update state variables with filtered data
        self.state["position_x"], self.state["position_y"] = state_mean[:2]


    def filter_orientation_data(self, orientation_data, timestamp):
        """
        Filter orientation data using Kalman filter and log data.
        """
        # Filter orientation data using Kalman filter
        state_mean, state_covariance = self.orientation_filter.filter_update(
            self.orientation_filter.initial_state_mean,
            self.orientation_filter.initial_state_covariance,
            orientation_data  # Pass noisy data directly
        )

        # Update state variables with filtered data
        self.state["orientation_x"], self.state["orientation_y"], self.state["orientation_z"], self.state["orientation_w"] = state_mean
        
        # Calculate yaw from filtered orientation data
        quaternion_filtered = [self.state["orientation_x"], self.state["orientation_y"], self.state["orientation_z"], self.state["orientation_w"]]
        yaw_after = euler_from_quaternion(quaternion_filtered)[2]

        # Log filtered orientation data to CSV
        with open(self.csv_filtered_path, 'a', newline='') as f:
            csv.writer(f).writerow([timestamp, self.state["position_x"], self.state["position_y"], yaw_after])

    def filter_velocity_data(self, velocity_data):
        """
        Filter linear velocity data using Kalman filter.
        """
        # Filter velocity data using Kalman filter
        state_mean, state_covariance = self.velocity_filter.filter_update(
            self.velocity_filter.initial_state_mean,
            self.velocity_filter.initial_state_covariance,
            velocity_data  # Pass noisy data directly
        )

        # Update state variables with filtered data
        self.state["linear_x"], self.state["linear_y"], self.state["linear_z"] = state_mean

    def publish_filtered_odom(self, timestamp):
        """
        Publish filtered odometry data.
        """
        # Create a new Odometry message for the filtered data
        filtered_odom_msg = Odometry()
        filtered_odom_msg.header.stamp = rospy.Time.now()
        filtered_odom_msg.header.frame_id = 'odom'

        # Set the filtered position
        filtered_odom_msg.pose.pose.position.x = self.state["position_x"]
        filtered_odom_msg.pose.pose.position.y = self.state["position_y"]

        # Set the filtered orientation
        filtered_odom_msg.pose.pose.orientation.x = self.state["orientation_x"]
        filtered_odom_msg.pose.pose.orientation.y = self.state["orientation_y"]
        filtered_odom_msg.pose.pose.orientation.z = self.state["orientation_z"]
        filtered_odom_msg.pose.pose.orientation.w = self.state["orientation_w"]

        # Set the filtered linear velocity
        filtered_odom_msg.twist.twist.linear.x = self.state["linear_x"]
        filtered_odom_msg.twist.twist.linear.y = self.state["linear_y"]
        filtered_odom_msg.twist.twist.linear.z = self.state["linear_z"]
        
        # Publish filtered odometry data
        self.filtered_pub.publish(filtered_odom_msg)

    def close_csv_files(self):
        # No need to close files, as we use context managers with 'with'
        pass

    def log_callback_odom(self, msg):
        """
        Callback function for odometry messages that injects Gaussian noise into read data.
        """
        # Extract position, orientation, and velocity data from the incoming message
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        linear_velocity = msg.twist.twist.linear
        # Log original data to CSV

        # Compute yaw from orientation data (quaternion) with noise
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        yaw = euler_from_quaternion(quaternion)[2]

        timestamp = msg.header.stamp.to_sec()
        with open(self.csv_before_path, 'a', newline='') as f:
            csv.writer(f).writerow([timestamp, position.x, position.y, yaw])

        # Parameters for Gaussian noise
        pos_noise_std_dev = 0.1  # Standard deviation for position noise
        ori_noise_std_dev = 0.01  # Standard deviation for orientation noise
        vel_noise_std_dev = 0.05  # Standard deviation for velocity noise
        
        # Adding Gaussian noise to the data
        noisy_position_x = position.x + np.random.normal(0, pos_noise_std_dev)
        noisy_position_y = position.y + np.random.normal(0, pos_noise_std_dev)
        noisy_orientation_x = orientation.x + np.random.normal(0, ori_noise_std_dev)
        noisy_orientation_y = orientation.y + np.random.normal(0, ori_noise_std_dev)
        noisy_orientation_z = orientation.z + np.random.normal(0, ori_noise_std_dev)
        noisy_orientation_w = orientation.w + np.random.normal(0, ori_noise_std_dev)
        noisy_linear_velocity_x = linear_velocity.x + np.random.normal(0, vel_noise_std_dev)
        noisy_linear_velocity_y = linear_velocity.y + np.random.normal(0, vel_noise_std_dev)
        noisy_linear_velocity_z = linear_velocity.z + np.random.normal(0, vel_noise_std_dev)

        # Compute yaw from orientation data (quaternion) with noise
        noisy_quaternion = [noisy_orientation_x, noisy_orientation_y, noisy_orientation_z, noisy_orientation_w]
        yaw_before = euler_from_quaternion(noisy_quaternion)[2]

        # Log noised orientation data to CSV
        with open(self.csv_noise_path, 'a', newline='') as f:
            csv.writer(f).writerow([timestamp, noisy_position_x, noisy_position_y, yaw_before])

        # Process and filter noisy data
        self.filter_position_data(np.array([noisy_position_x, noisy_position_y]), timestamp)
        self.filter_orientation_data(np.array([noisy_orientation_x, noisy_orientation_y, noisy_orientation_z, noisy_orientation_w]), timestamp)
        self.filter_velocity_data(np.array([noisy_linear_velocity_x, noisy_linear_velocity_y, noisy_linear_velocity_z]))
        
        # Publish the filtered odometry data
        self.publish_filtered_odom(timestamp)

def main():
    rospy.init_node('filtered_odom_publisher', anonymous=True)
    
    odom_handler = OdometryHandler()
    
    rospy.on_shutdown(odom_handler.close_csv_files)
    rospy.spin()

if __name__ == "__main__":
    main()