#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
import sensor_msgs.point_cloud2 as pc2
import math


class LidarSubscriber:
    def __init__(self):
        rospy.init_node('lidar_subscriber', anonymous=True)
        self.lidar_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.lidar_callback)
        self.distance_pub = rospy.Publisher("/lidar_distance", Float32, queue_size=10)
        self.no_object_distance = 0.0
        # Publish initial value of 0.0
        self.distance_pub.publish(self.no_object_distance)

    def lidar_callback(self, data):
        # rospy.logdebug("Received LiDAR data")
        closest_distance = float('inf')
        object_detected = False

        for point in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point

            # Assuming the car is moving in the positive x direction
            if 60 >= y >= 2 >= x >= -2 and z >= -1.2:  # Adjust detection zone as needed

                # Calculate distance using Euclidean distance formula
                distance = math.sqrt(x ** 2 + y ** 2 + z ** 2)

                # Update the closest distance if within range
                if distance >= 1:
                    object_detected = True
                    if distance < closest_distance:
                        closest_distance = distance

        if object_detected:
            self.distance_pub.publish(closest_distance)
        else:
            self.distance_pub.publish(self.no_object_distance)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = LidarSubscriber()
        node.run()
    except rospy.ROSInterruptException:
        pass
