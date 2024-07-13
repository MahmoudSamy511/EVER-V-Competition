#-----------------------------------------------------------------------------
import rospy
import json
from std_msgs.msg import Bool, Float32, String

class ObstacleHandlingNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('obstacle_handling_node', anonymous=True)

        # Subscribers
        rospy.Subscriber('/lidar_distance', Float32, self.lidar_distance_callback)
        rospy.Subscriber('/detected_object_labels', String, self.object_labels_callback)
        rospy.Subscriber('/is_dynamic_obstacle', Bool, self.dynamic_obstacle_callback)

        # Publishers
        self.actions_pub = rospy.Publisher("/situations" , String, queue_size=10)

        # Internal state
        self.is_dynamic_obstacle = False
        self.left_lane_obstacle = None
        self.center_obstacle = None
        self.right_lane_obstacle = None

    def lidar_distance_callback(self, msg):
        lidar_distance = msg.data
        rospy.loginfo("-----------------------------")
        rospy.loginfo(f"Lidar distance received: {lidar_distance}")
        if lidar_distance > 30:  # adjustable
            self.actions_pub.publish('All Good')
            rospy.loginfo("all good")
        else:
            self.handle_obstacle()

    def handle_obstacle(self):
        if self.is_dynamic_obstacle:
            rospy.loginfo("Published True to /slowdown")
            self.actions_pub.publish('Gradually Stops')

            self.handle_dynamic_obstacle()
        else:
            self.handle_static_obstacle()

    def handle_dynamic_obstacle(self):
        if self.left_lane_obstacle == 'person' or self.center_obstacle == 'person' or self.right_lane_obstacle == 'person':
            self.handle_human_obstacle_dynamic()
        elif self.center_obstacle == 'car':
            self.handle_car_obstacle()
        else:
            self.actions_pub.publish('Gradually Stops')
            rospy.loginfo("slowdown")

    def handle_car_obstacle(self):
        if self.left_lane_obstacle == 'None' and self.right_lane_obstacle == 'None':
            self.actions_pub.publish('Lane Change to the right')
            rospy.loginfo("Published True to /change_lane_to_right")
        elif self.left_lane_obstacle == 'None':
            self.actions_pub.publish('Lane Change to the left')
            rospy.loginfo("Published True to /change_lane_to_left")
        elif self.right_lane_obstacle == 'None':
            self.actions_pub.publish('Lane Change to the right')
            rospy.loginfo("Published True to /change_lane_to_right")
        else:
            self.actions_pub.publish('Adaptive Cruise Control')
            rospy.loginfo("Adaptive Cruise Control")

    def handle_human_obstacle_dynamic(self):
        self.actions_pub.publish('Emergency Stops')
        rospy.loginfo("emergency_break")

    def handle_static_obstacle(self):
        if self.left_lane_obstacle == 'person' or self.center_obstacle == 'person' or self.right_lane_obstacle == 'person':
            self.actions_pub.publish('Emergency Stops')
            rospy.loginfo("Published True to /emergency_break")
        elif self.left_lane_obstacle == 'None' and self.right_lane_obstacle == 'None':
            self.actions_pub.publish('Lane Change to the right')
            rospy.loginfo("Published True to /change_lane_to_right")
        elif self.left_lane_obstacle == 'None':
            self.actions_pub.publish('Lane Change to the left')
            rospy.loginfo("Published True to /change_lane_to_left")
        elif self.right_lane_obstacle == 'None':
            self.actions_pub.publish('Lane Change to the right')
            rospy.loginfo("Published True to /change_lane_to_right")
        elif self.left_lane_obstacle == 'car' and self.center_obstacle == 'car' and self.right_lane_obstacle == 'car':
            self.actions_pub.publish('Emergency Stops')
            rospy.loginfo("Published True to /emergency_break")
        else:
            self.actions_pub.publish('Gradually Stops')
            rospy.loginfo("slowdown")

    def object_labels_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.left_lane_obstacle = data.get('left lane')
            self.center_obstacle = data.get('center')
            self.right_lane_obstacle = data.get('right lane')
            
            rospy.loginfo(f"Left lane obstacle: {self.left_lane_obstacle}")
            rospy.loginfo(f"Center obstacle: {self.center_obstacle}")
            rospy.loginfo(f"Right lane obstacle: {self.right_lane_obstacle}")
        except json.JSONDecodeError:
            rospy.logerr("Failed to decode JSON from detected_object_labels")

    def dynamic_obstacle_callback(self, msg):
        self.is_dynamic_obstacle = msg.data
        rospy.loginfo(f"Is dynamic obstacle: {self.is_dynamic_obstacle}")

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ObstacleHandlingNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

