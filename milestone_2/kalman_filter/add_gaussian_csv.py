import rospy
from nav_msgs.msg import Odometry
import pandas as pd
import numpy as np
import csv

data_storage = []

def add_gaussian_noise(data, mean=0, std_dev=0.1):
    """Add Gaussian noise to the data."""
    noise = np.random.normal(mean, std_dev, len(data))
    return data + noise

def odom_callback(data):
    # extract x, y and yaw coordinates from Odometry data
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    # yaw represents orientation in 2D. It is extracted from quaternion representation of orientation
    yaw = 2 * np.arctan2(data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    
    # store into list
    data_storage.append([x, y, yaw])

def listener():
    rospy.init_node('odom_listener', anonymous=True)

    rospy.Subscriber("/odom", Odometry, odom_callback)

    # keep python from exiting until this node is stopped
    rospy.spin()

def main():
    listener()

    # Convert the data list to a pandas DataFrame.
    data = pd.DataFrame(data_storage, columns=["X", "Y", "Yaw"])

    # File path of the output data file (with Gaussian noise added)
    output_file = 'odom_noisy_xyz.csv'
    
    # Add Gaussian noise to the X, Y, and Yaw columns
    data['X'] = add_gaussian_noise(data['X'])
    data['Y'] = add_gaussian_noise(data['Y'])
    data['Yaw'] = add_gaussian_noise(data['Yaw'])
    
    # Save the noisy data to a new CSV file
    data.to_csv(output_file, index=False)
  
if __name__ == '__main__':
    main()