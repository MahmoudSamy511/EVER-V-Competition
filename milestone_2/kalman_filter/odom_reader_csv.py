import rospy
from nav_msgs.msg import Odometry
import csv

data_storage = []

def odom_callback(data):
    # extract x, y coordinates from Odometry data
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    
    # store into list
    data_storage.append((x, y))

def listener():
    rospy.init_node('odom_listener', anonymous=True)

    rospy.Subscriber("/odom", Odometry, odom_callback)

    # keep python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

    # when node is stopped, write to csv
    with open('odom_data.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y'])  # writing the header

        for x, y in data_storage:
            writer.writerow([x, y])  # writing the data