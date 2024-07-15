#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import Bool, Float32, String


class ObstacleHandlingNode:
    current_lane = [0, 1, 0]
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('obstacle_handling_node', anonymous=True)

        # Subscribers
        rospy.Subscriber('/lidar_distance', Float32, self.lidar_distance_callback)
        rospy.Subscriber('/detected_object_labels', String, self.object_labels_callback)
        rospy.Subscriber('/is_dynamic_obstacle', Bool, self.dynamic_obstacle_callback)
        rospy.Subscriber('/is_left_lane_free', Bool, self.leftLane_callback)
        rospy.Subscriber('/is_right_lane_free', Bool, self.rightLane_callback)


        # Publishers
        self.actions_pub = rospy.Publisher("/situations" , String, queue_size=10)
        # Internal state
        self.is_dynamic_obstacle = False
        self.left_lane_obstacle = None #'None'
        self.center_obstacle =  None #'None'
        self.right_lane_obstacle =  None #'None'
        self.is_left_lane_free = False
        self.is_right_lane_free = False

        
    def lidar_distance_callback(self, msg):
        lidar_distance = msg.data
        if lidar_distance > 15 or lidar_distance == 0.0 :  # adjustable
            self.actions_pub.publish('All Good')
        else:
            self.actions_pub.publish('Gradually Stops')
            self.handle_obstacle()

    def handle_obstacle(self):
        if self.is_dynamic_obstacle:
            # self.actions_pub.publish('Gradually Stops')

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

    def handle_car_obstacle(self):
        # if self.left_lane_obstacle == None:
        if self.is_left_lane_free and (self.center_obstacle!='person' and self.left_lane_obstacle!='person'):
            if self.current_lane[0] != 1 :
                self.actions_pub.publish('Lane Change to the left')
                if self.current_lane[2]:
                    self.current_lane[0], self.current_lane[1], self.current_lane[2] = 0, 1, 0
                elif self.current_lane[1]:
                    self.current_lane[0], self.current_lane[1], self.current_lane[2] = 1, 0, 0
            else:
                # if self.right_lane_obstacle == None:
                if self.is_right_lane_free and (self.center_obstacle!='person' and self.right_lane_obstacle!='person'):
                    self.actions_pub.publish('Lane Change to the right')
                    if self.current_lane[1]:
                        self.current_lane[0], self.current_lane[1], self.current_lane[2] = 0, 0, 1
                
                    elif self.current_lane[0]:
                        self.current_lane[0], self.current_lane[1], self.current_lane[2] = 0, 1, 0
                else:
                    self.actions_pub.publish('Emergency Stops')

        elif self.right_lane_obstacle == None:
            if self.current_lane[2] != [1] :
                self.actions_pub.publish('Lane Change to the right')
                if self.current_lane[1]:
                    self.current_lane = [0, 0, 1]
            
                elif self.current_lane[0]:
                    self.current_lane = [0, 1, 0] 
            else:
                if self.left_lane_obstacle == None:
                    self.actions_pub.publish('Lane Change to the left')
                    if self.current_lane[2]:
                        self.current_lane = [0,1,0]
                    elif self.current_lane[1]:
                        self.current_lane=[1,0,0]
                else:
                    self.actions_pub.publish('Emergency Stops')
        
        else:
            self.actions_pub.publish('Adaptive Cruise Control')

    def handle_human_obstacle_dynamic(self):
        self.actions_pub.publish('Emergency Stops')

    def handle_static_obstacle(self):
        # self.actions_pub.publish('Gradually Stops')
        if self.left_lane_obstacle == 'person' or self.center_obstacle == 'person' or self.right_lane_obstacle == 'person':
            self.actions_pub.publish('Emergency Stops')
        
        # elif self.left_lane_obstacle == None:
        elif self.is_left_lane_free:
            if self.current_lane[0] != 1:
                self.actions_pub.publish('Lane Change to the left')
                if self.current_lane[2] == 1:
                    self.current_lane = [0, 1, 0]
                elif self.current_lane[1] == 1:
                    self.current_lane = [1, 0, 0]
            else:
                if self.right_lane_obstacle == None:
                    self.actions_pub.publish('Lane Change to the right')
                    if self.current_lane[1] == 1:
                        self.current_lane = [0, 0, 1]
                    elif self.current_lane[0]:
                        self.current_lane = [0, 1, 0]
                else:
                    self.actions_pub.publish('Emergency Stops')
        
        elif self.right_lane_obstacle == None:
            if self.current_lane[2] != 1 : 
                self.actions_pub.publish('Lane Change to the right')
                if self.current_lane[1]:
                        self.current_lane = [0, 0, 1]
                elif self.current_lane[0]:
                    self.current_lane = [0, 1, 0]

        elif self.left_lane_obstacle == 'car' or self.center_obstacle == 'car' or self.right_lane_obstacle == 'car':
            self.actions_pub.publish('Emergency Stops')
        
        else:
            self.actions_pub.publish('Gradually Stops')

    def leftLane_callback(self, msg):
        self.is_left_lane_free = msg.data
    def rightLane_callback(self, msg):
        self.is_right_lane_free = msg.data

    def object_labels_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.left_lane_obstacle = data.get('left lane')
            self.center_obstacle = data.get('center')
            self.right_lane_obstacle = data.get('right lane')
            
        except json.JSONDecodeError:
            rospy.logerr("Failed to decode JSON from detected_object_labels")

    def dynamic_obstacle_callback(self, msg):
        self.is_dynamic_obstacle = msg.data
        

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ObstacleHandlingNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass