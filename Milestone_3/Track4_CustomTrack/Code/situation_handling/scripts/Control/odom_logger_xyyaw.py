import rospy
from nav_msgs.msg import Odometry
import csv
from tf.transformations import euler_from_quaternion

class OdometryLogger:
    """
    Class to handle odometry data and log position data in CSV file.
    """
    def __init__(self):
        # Initialize the CSV file for logging position data
        self.csv_file = open('logging/odom_position_data.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write the header row to the CSV file
        self.csv_writer.writerow(['x', 'y', 'yaw'])
        
        # Initialize subscriber for odometry messages
        rospy.Subscriber('/odom', Odometry, self.log_callback_odom)
        
    def log_callback_odom(self, msg):
        """
        Callback function for odometry messages.
        Logs the x, y positions, and yaw to the CSV file.
        """
        # Extract x and y positions from the odometry message
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        
        # Get the current timestamp
        timestamp = rospy.Time.now().to_sec()
        
        # Extract orientation quaternion from the odometry message
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        
        # Convert orientation from quaternion to Euler angles
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        # Write the timestamp, position_x, position_y, and yaw to the CSV file
        self.csv_writer.writerow([position_x, position_y, yaw])
    
    def __del__(self):
        # Close the CSV file when the object is destroyed
        if self.csv_file:
            self.csv_file.close()

def main():
    # Initialize the ROS node
    rospy.init_node('odometry_logger', anonymous=True)
    
    # Create an OdometryLogger object
    odometry_logger = OdometryLogger()
    
    # Keep the node running
    rospy.spin()
    
if __name__ == '__main__':
    main()
