import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class OdometryPlotter:
    def __init__(self):
        # Initialize lists to store data for plotting
        self.odom_position_x = []
        self.odom_position_y = []
        self.filtered_odom_position_x = []
        self.filtered_odom_position_y = []
        
        self.odom_velocity_x = []
        self.filtered_odom_velocity_x = []
        
        # Initialize ROS node
        rospy.init_node('odom_plotter', anonymous=True)
        
        # Create subscribers for odom and filtered_odom topics
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/filtered_odom', Odometry, self.filtered_odom_callback)
        
        # Set up plot
        self.fig, self.axs = plt.subplots(2, 1, figsize=(8, 10))
        
        # Initialize plot lines
        self.position_line, = self.axs[0].plot([], [], 'b-', label='Position - /odom')
        self.filtered_position_line, = self.axs[0].plot([], [], 'r-', label='Position - /filtered_odom')
        self.velocity_line, = self.axs[1].plot([], [], 'b-', label='Velocity - /odom')
        self.filtered_velocity_line, = self.axs[1].plot([], [], 'r-', label='Velocity - /filtered_odom')
        
        # Set plot titles and labels
        self.axs[0].set_title('Position Data')
        self.axs[0].set_xlabel('x position')
        self.axs[0].set_ylabel('y position')
        self.axs[1].set_title('Linear Velocity Data')
        self.axs[1].set_xlabel('Time')
        self.axs[1].set_ylabel('Velocity (m/s)')
        
        # Add legends
        self.axs[0].legend()
        self.axs[1].legend()
        
        # Set up plot update function
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)
        
        # Show plot
        plt.show()
        
    def odom_callback(self, msg):
        """
        Callback function for the /odom topic.
        """
        # Extract position and velocity data from the /odom message
        self.odom_position_x.append(msg.pose.pose.position.x)
        self.odom_position_y.append(msg.pose.pose.position.y)
        
        self.odom_velocity_x.append(msg.twist.twist.linear.x)
        
    def filtered_odom_callback(self, msg):
        """
        Callback function for the /filtered_odom topic.
        """
        # Extract position and velocity data from the /filtered_odom message
        self.filtered_odom_position_x.append(msg.pose.pose.position.x)
        self.filtered_odom_position_y.append(msg.pose.pose.position.y)
        
        self.filtered_odom_velocity_x.append(msg.twist.twist.linear.x)
        
    def update_plot(self, frame):
        """
        Update function for the plot animation.
        """
        # Update position data plot
        self.position_line.set_data(self.odom_position_x, self.odom_position_y)
        self.filtered_position_line.set_data(self.filtered_odom_position_x, self.filtered_odom_position_y)
        
        # Update velocity data plot
        self.velocity_line.set_data(range(len(self.odom_velocity_x)), self.odom_velocity_x)
        self.filtered_velocity_line.set_data(range(len(self.filtered_odom_velocity_x)), self.filtered_odom_velocity_x)
        
        # Adjust the plot limits
        self.axs[0].relim()
        self.axs[0].autoscale_view()
        
        self.axs[1].relim()
        self.axs[1].autoscale_view()
        
        return self.position_line, self.filtered_position_line, self.velocity_line, self.filtered_velocity_line
        
if __name__ == '__main__':
    # Create an OdometryPlotter instance
    plotter = OdometryPlotter()
    
    # Run the ROS node
    rospy.spin()
