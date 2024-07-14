#!/usr/bin/env python3



import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import math
from std_msgs.msg import String  # Import String message type


class LidarSubscriber:
    def __init__(self):
        rospy.init_node('lidar_subscriber', anonymous=True)
        self.lidar_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.lidar_callback)
        self.last_distance = None
        self.reading_count = 0
        self.is_dynamic_pub = rospy.Publisher("is_dynamic_obstacle", String, queue_size=10)

    def lidar_callback(self, data):
        # rospy.logdebug("Received LiDAR data")

        for point in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point

            # Assuming the car is moving in the positive x direction
            if 2 <= y <= 60 and -2 <= x <= 2 and z >= -1.2:  # Adjust detection zone as needed

                # Calculate distance using Euclidean distance formula
                distance = math.sqrt(x ** 2 + y ** 2 + z ** 2)

                # Print x, y, z coordinates and their distance only
                if distance >= 1:
                    # rospy.loginfo(f"Distance to object: {distance:.2f} meters")

                    self.reading_count += 1

                    # Ignore the first 25 readings
                    if self.reading_count <= 25:
                        continue

                    # Calculate difference from last detected distance
                    if self.last_distance is not None:
                        distance_diff = distance - self.last_distance
                        if distance_diff <= 0.01:
                            # rospy.loginfo("Static")
                            self.publish_status("static")
                        else:
                            # rospy.loginfo(f"Dynamic .. Velocity: {distance_diff:.2f} m/s")
                            self.publish_status("dynamic")

                    self.last_distance = distance
                    return

    def publish_status(self, status):
        # Publish the status ("dynamic" or "static") under the topic "is_dynamic_obstacle"
        if status == "dynamic":
            status = "True"
        else:
            status = "False"
        self.is_dynamic_pub.publish(status)

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