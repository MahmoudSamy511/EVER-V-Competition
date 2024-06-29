import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
import sensor_msgs.point_cloud2 as pc2

class ObstacleDetector:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)
        self.steering_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
        self.brake_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
        self.obstacles = []

    def lidar_callback(self, data):
        cloud_points = list(pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z")))
        self.detect_obstacles(cloud_points)
        self.classify_and_react()

    def detect_obstacles(self, points):
        self.obstacles = []
        for p in points:
            distance = np.sqrt(p[0]**2 + p[1]**2 + p[2]**2)
            if distance < 10:  # Detect obstacles within 10 meters
                self.obstacles.append(p)

    def classify_and_react(self):
        for obs in self.obstacles:
            # Example classification logic based on position (simplified)
            if obs[0] > 0:  # Assume obstacles in front of the vehicle
                rospy.loginfo(f"Obstacle detected at {obs}")
                self.react_to_obstacle(obs)

    def react_to_obstacle(self, obs):
        # Example reaction logic
        angle = 0.0
        velocity = 0.5
        brake_pressure = 0.0

        if obs[1] > 0:  # Obstacle is to the right
            angle = -15.0  # Turn left
        elif obs[1] < 0:  # Obstacle is to the left
            angle = 15.0  # Turn right

        if np.sqrt(obs[0]**2 + obs[1]**2) < 5:  # If obstacle is very close
            velocity = 0.0
            brake_pressure = 1.0

        self.steering_pub.publish(Float64(angle))
        self.cmd_vel_pub.publish(Float64(velocity))
        self.brake_pub.publish(Float64(brake_pressure))

def main():
    rospy.init_node('obstacle_detection_classification')
    detector = ObstacleDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
