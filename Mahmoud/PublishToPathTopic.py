#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv
import time

def read_csv_file(file_path):
    path = Path()
    path.header.frame_id = "map"  # Set your desired frame ID
    with open(file_path, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        for row in csv_reader:
            pose = PoseStamped()
            pose.pose.position.x = float(row[0])  # Assuming the CSV has x-coordinate in the first column
            pose.pose.position.y = float(row[1])  # Assuming the CSV has y-coordinate in the second column
            pose.pose.position.z = 0.0  # Assuming z-coordinate is always 0
            # You can add orientation if available
            path.poses.append(pose)
    return path

if __name__ == "__main__":
    rospy.init_node("csv_to_path_publisher")
    pub = rospy.Publisher("/pure_pursuit/path", Path, queue_size=10)

    # Specify the path to your CSV file
    csv_file_path = "square_path copy.csv"

    path_msg = read_csv_file(csv_file_path)

    rospy.loginfo("Publishing path from CSV file row by row...")

    for pose in path_msg.poses:
        path_row = Path()
        path_row.header.frame_id = path_msg.header.frame_id
        path_row.header.stamp = rospy.Time.now()  # Update the timestamp
        path_row.poses.append(pose)
        pub.publish(path_row)
        time.sleep(1)  # Sleep to introduce a delay of 1 second between each row

    rospy.loginfo("Finished publishing path from CSV file.")

    rospy.spin()
