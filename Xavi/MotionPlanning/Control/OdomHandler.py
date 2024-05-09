from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from std_msgs.msg import Header
import rospy
import numpy as np
from scipy.ndimage import gaussian_filter1d
from filterpy.kalman import KalmanFilter
from tf.transformations import euler_from_quaternion
import math

class OdometryHandler:
    """
    Class to handle odometry data and republish filtered data.
    """
    def __init__(self):
        # Initialize state variables
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_x = 0.0
        self.orientation_y = 0.0
        self.orientation_z = 0.0
        self.orientation_w = 0.0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        
        # Initialize subscriber for odometry messages
        rospy.Subscriber('/odom', Odometry, self.log_callback_odom)
        
        # Initialize publisher for filtered odometry messages
        self.filtered_odom_pub = rospy.Publisher('/filtered_odom', Odometry, queue_size=10)
        
        # Initialize separate Kalman filters for position, orientation, and linear velocity
        # Kalman filter for position (x, y)
        self.position_filter = KalmanFilter(dim_x=2, dim_z=2)
        self.position_filter.x = np.array([self.position_x, self.position_y])
        self.position_filter.P = np.eye(2) * 1.0
        self.position_filter.Q = np.eye(2) * 0.01
        self.position_filter.R = np.eye(2) * 0.1
        self.position_filter.F = np.eye(2)  # Assuming static position
        self.position_filter.H = np.eye(2)
        
        # Kalman filter for orientation (x, y, z, w)
        self.orientation_filter = KalmanFilter(dim_x=4, dim_z=4)
        self.orientation_filter.x = np.array([self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w])
        self.orientation_filter.P = np.eye(4) * 1.0
        self.orientation_filter.Q = np.eye(4) * 0.01
        self.orientation_filter.R = np.eye(4) * 0.1
        self.orientation_filter.F = np.eye(4)  # Assuming static orientation
        self.orientation_filter.H = np.eye(4)
        
        # Kalman filter for linear velocity (x, y, z)
        self.velocity_filter = KalmanFilter(dim_x=3, dim_z=3)
        self.velocity_filter.x = np.array([self.linear_x, self.linear_y, self.linear_z])
        self.velocity_filter.P = np.eye(3) * 1.0
        self.velocity_filter.Q = np.eye(3) * 0.01
        self.velocity_filter.R = np.eye(3) * 0.1
        self.velocity_filter.F = np.eye(3)  # Assuming constant velocity
        self.velocity_filter.H = np.eye(3)
        
    def log_callback_odom(self, msg):
        """
        Callback function for odometry messages.
        """
        # Extract and smooth position data using a Gaussian filter
        position = msg.pose.pose.position
        position_data = np.array([position.x, position.y])
        smoothed_position = gaussian_filter1d(position_data, sigma=0.5)
        
        # Update Kalman filter with smoothed position data
        self.position_filter.predict()
        self.position_filter.update(smoothed_position)
        self.position_x, self.position_y = self.position_filter.x
        
        # Extract and smooth orientation data using a Gaussian filter
        orientation = msg.pose.pose.orientation
        orientation_data = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        smoothed_orientation = gaussian_filter1d(orientation_data, sigma=0.5)
        
        # Update Kalman filter with smoothed orientation data
        self.orientation_filter.predict()
        self.orientation_filter.update(smoothed_orientation)
        self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w = self.orientation_filter.x
        
        # Extract and smooth linear velocity data using a Gaussian filter
        twist = msg.twist.twist.linear
        velocity_data = np.array([twist.x, twist.y, twist.z])
        smoothed_velocity = gaussian_filter1d(velocity_data, sigma=0.5)
        
        # Update Kalman filter with smoothed velocity data
        self.velocity_filter.predict()
        self.velocity_filter.update(smoothed_velocity)
        self.linear_x, self.linear_y, self.linear_z = self.velocity_filter.x
        
        # Create a new Odometry message for the filtered data
        filtered_odom_msg = Odometry()
        filtered_odom_msg.header.stamp = rospy.Time.now()
        filtered_odom_msg.header.frame_id = 'odom'
        
        # Set the filtered position
        filtered_odom_msg.pose.pose.position.x = self.position_x
        filtered_odom_msg.pose.pose.position.y = self.position_y
        
        # Set the filtered orientation
        filtered_odom_msg.pose.pose.orientation.x = self.orientation_x
        filtered_odom_msg.pose.pose.orientation.y = self.orientation_y
        filtered_odom_msg.pose.pose.orientation.z = self.orientation_z
        filtered_odom_msg.pose.pose.orientation.w = self.orientation_w
        
        # Set the filtered linear velocity
        filtered_odom_msg.twist.twist.linear.x = self.linear_x
        filtered_odom_msg.twist.twist.linear.y = self.linear_y
        filtered_odom_msg.twist.twist.linear.z = self.linear_z
        
        # Publish the filtered odometry message
        self.filtered_odom_pub.publish(filtered_odom_msg)

def main():
    # Initialize the ROS node
    rospy.init_node('filtered_odom_publisher', anonymous=True)
    
    # Create an OdometryHandler object
    odom_handler = OdometryHandler()
    
    # Keep the node running
    rospy.spin()
    
if __name__ == '__main__':
    main()

