#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
import sensor_msgs.point_cloud2 as pc2
import math

class LidarSubscriber:
    def __init__(self):
        rospy.init_node('lidar_subscriber', anonymous=True)
        self.lidar_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.lidar_callback)
        self.distance_front = rospy.Publisher('/distance_front', Float64, queue_size=10)

    def lidar_callback(self, data):
        rospy.logdebug("Received LiDAR data")

        for point in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            
            # Assuming the car is moving in the positive x direction
            if 2 <= y <= 60 and -2 <= x <= 2 and z>=-1.2:  # Adjust detection zone as needed
                
                # Calculate distance using Euclidean distance formula
                distance = math.sqrt(x**2 + y**2 + z**2)
                
                # Print x, y, z coordinates and their distance only
                if distance >=1:
                    self.distance_front.publish(distance)
                    rospy.loginfo(f"Distance: {distance} meters")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LidarSubscriber()
        rospy.loginfo("LidarSubscriber node initialized")
        rospy.loginfo("Waiting for LiDAR data...")
        node.run()
    except rospy.ROSInterruptException:
        pass
