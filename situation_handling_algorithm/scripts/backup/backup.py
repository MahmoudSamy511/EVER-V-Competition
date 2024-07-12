# #!/usr/bin/env python

# import rospy
# from std_msgs.msg import Bool, Float32, String

# class ObstacleHandlingNode:
#     def __init__(self):
#         # Initialize the ROS node
#         rospy.init_node('obstacle_handling_node', anonymous=True)

#         # Subscribers
#         rospy.Subscriber('/lidar_distance', Float32, self.lidar_distance_callback)
#         rospy.Subscriber('/detected_object_labels', String, self.object_labels_callback)
#         rospy.Subscriber('/is_dynamic_obstacle', Bool, self.dynamic_obstacle_callback)
#         rospy.Subscriber('/is_left_lane_free', Bool, self.left_lane_free_callback)
#         rospy.Subscriber('/is_right_lane_free', Bool, self.right_lane_free_callback)

#         # Publishers
#         self.pub_stay_in_lane = rospy.Publisher('/stay_in_lane', Bool, queue_size=10)
#         self.pub_slowdown = rospy.Publisher('/slowdown', Bool, queue_size=10)
#         self.pub_change_lane_to_right = rospy.Publisher('/change_lane_to_right', Bool, queue_size=10)
#         self.pub_change_lane_to_left = rospy.Publisher('/change_lane_to_left', Bool, queue_size=10)
#         self.pub_maintain_desired_distance = rospy.Publisher('/maintain_desired_distance', Bool, queue_size=10)
#         self.pub_emergency_break = rospy.Publisher('/emergency_break', Bool, queue_size=10)

#         # Internal state
#         self.obstacle_type = None
#         self.is_dynamic_obstacle = None
#         self.is_left_lane_free = None
#         self.is_right_lane_free = None

#     def lidar_distance_callback(self, msg):
#         lidar_distance = msg.data
#         if lidar_distance > 40:  # adjustable
#             self.pub_stay_in_lane.publish(True)
#         else:
#             self.handle_obstacle()

#     def handle_obstacle(self):
#         if self.is_dynamic_obstacle:
#             self.handle_dynamic_obstacle()
#         else:
#             self.handle_static_obstacle()

#     def handle_dynamic_obstacle(self):
#         if self.obstacle_type == 'human':
#             self.handle_human_obstacle_dynamic()
#         elif self.obstacle_type == 'car':
#             self.handle_car_obstacle()
#         else:
#             self.handle_unknown_dynamic_obstacle()

#     def handle_car_obstacle(self):
#         if self.is_left_lane_free and self.is_right_lane_free:
#             self.pub_change_lane_to_right.publish(True)
#         elif self.is_left_lane_free:
#             self.pub_change_lane_to_left.publish(True)
#         elif self.is_right_lane_free:
#             self.pub_change_lane_to_right.publish(True)
#         else:
#             self.pub_maintain_desired_distance.publish(True)

#     def handle_human_obstacle_dynamic(self):
#         self.pub_emergency_break.publish(True)

#     def handle_static_obstacle(self):
#         if self.is_left_lane_free and self.is_right_lane_free:
#             self.pub_change_lane_to_right.publish(True)
#         elif self.is_left_lane_free:
#             self.pub_change_lane_to_left.publish(True)
#         elif self.is_right_lane_free:
#             self.pub_change_lane_to_right.publish(True)
#         else:
#             self.pub_emergency_break.publish(True)

#     def handle_unknown_dynamic_obstacle(self):
#         self.pub_emergency_break.publish(True)

#     def object_labels_callback(self, msg):
#         self.obstacle_type = msg.data

#     def dynamic_obstacle_callback(self, msg):
#         self.is_dynamic_obstacle = msg.data

#     def left_lane_free_callback(self, msg):
#         self.is_left_lane_free = msg.data

#     def right_lane_free_callback(self, msg):
#         self.is_right_lane_free = msg.data

#     def spin(self):
#         rospy.spin()

# if __name__ == '__main__':
#     try:
#         node = ObstacleHandlingNode()
#         node.spin()
#     except rospy.ROSInterruptException:
#         pass


#--------------------------------------------------------------------------------------


#!/usr/bin/env python

# import rospy
# from std_msgs.msg import Bool, Float32, String

# class ObstacleHandlingNode:
#     def __init__(self):
#         # Initialize the ROS node
#         rospy.init_node('obstacle_handling_node', anonymous=True)

#         # Subscribers
#         rospy.Subscriber('/lidar_distance', Float32, self.lidar_distance_callback)
#         rospy.Subscriber('/detected_object_labels', String, self.object_labels_callback)
#         rospy.Subscriber('/is_dynamic_obstacle', Bool, self.dynamic_obstacle_callback)
#         rospy.Subscriber('/is_left_lane_free', Bool, self.left_lane_free_callback)
#         rospy.Subscriber('/is_right_lane_free', Bool, self.right_lane_free_callback)

#         # Publishers
#         self.pub_stay_in_lane = rospy.Publisher('/stay_in_lane', Bool, queue_size=10)
#         self.pub_slowdown = rospy.Publisher('/slowdown', Bool, queue_size=10)
#         self.pub_change_lane_to_right = rospy.Publisher('/change_lane_to_right', Bool, queue_size=10)
#         self.pub_change_lane_to_left = rospy.Publisher('/change_lane_to_left', Bool, queue_size=10)
#         self.pub_maintain_desired_distance = rospy.Publisher('/maintain_desired_distance', Bool, queue_size=10)
#         self.pub_emergency_break = rospy.Publisher('/emergency_break', Bool, queue_size=10)

#         # Internal state
#         self.obstacle_type = "Undefined"
#         self.is_dynamic_obstacle = "Undefined"
#         self.is_left_lane_free = False
#         self.is_right_lane_free = False

#     def lidar_distance_callback(self, msg):
#         lidar_distance = msg.data
#         rospy.loginfo("-----------------------------")
#         rospy.loginfo(f"Lidar distance received: {lidar_distance}")
#         if lidar_distance > 30:  # adjustable
#             self.pub_stay_in_lane.publish(True)
#             rospy.loginfo("Published True to /stay_in_lane")
#         else:
#             self.handle_obstacle()

#     def handle_obstacle(self):
#         if self.is_dynamic_obstacle:
#             self.pub_slowdown.publish(True)
#             rospy.loginfo("Published True to /Slowdown")
#             self.handle_dynamic_obstacle()
#         else:
#             self.handle_static_obstacle()

#     def handle_dynamic_obstacle(self):
#         if(self.obstacle_type != 'car' or 'human'):
#             self.pub_slowdown.publish(True)
#             rospy.loginfo("Published True to /Slowdown")
#         while(self.obstacle_type == 'human' or self.obstacle_type == 'car'):
#             if self.obstacle_type == 'human':
#                 self.handle_human_obstacle_dynamic()
#             elif self.obstacle_type == 'car':
#                 self.handle_car_obstacle()

#     def handle_car_obstacle(self):
#         if self.is_left_lane_free and self.is_right_lane_free:
#             self.pub_change_lane_to_right.publish(True)
#             rospy.loginfo("Published True to /change_lane_to_right")
#         elif self.is_left_lane_free:
#             self.pub_change_lane_to_left.publish(True)
#             rospy.loginfo("Published True to /change_lane_to_left")
#         elif self.is_right_lane_free:
#             self.pub_change_lane_to_right.publish(True)
#             rospy.loginfo("Published True to /change_lane_to_right")
#         else:
#             self.pub_maintain_desired_distance.publish(True)
#             rospy.loginfo("Published True to /maintain_desired_distance")

#     def handle_human_obstacle_dynamic(self):
#         self.pub_emergency_break.publish(True)
#         rospy.loginfo("Published True to /emergency_break")

#     def handle_static_obstacle(self):
#         if self.is_left_lane_free and self.is_right_lane_free:
#             self.pub_change_lane_to_right.publish(True)
#             rospy.loginfo("Published True to /change_lane_to_right")
#         elif self.is_left_lane_free:
#             self.pub_change_lane_to_left.publish(True)
#             rospy.loginfo("Published True to /change_lane_to_left")
#         elif self.is_right_lane_free:
#             self.pub_change_lane_to_right.publish(True)
#             rospy.loginfo("Published True to /change_lane_to_right")
#         else:
#             self.pub_emergency_break.publish(True)
#             rospy.loginfo("Published True to /emergency_break")

#     def object_labels_callback(self, msg):
#         self.obstacle_type = msg.data
#         rospy.loginfo(f"Detected object label: {self.obstacle_type}")

#     def dynamic_obstacle_callback(self, msg):
#         self.is_dynamic_obstacle = msg.data
#         rospy.loginfo(f"Is dynamic obstacle: {self.is_dynamic_obstacle}")

#     def left_lane_free_callback(self, msg):
#         self.is_left_lane_free = msg.data
#         rospy.loginfo(f"Is left lane free: {self.is_left_lane_free}")

#     def right_lane_free_callback(self, msg):
#         self.is_right_lane_free = msg.data
#         rospy.loginfo(f"Is right lane free: {self.is_right_lane_free}")

#     def spin(self):
#         rospy.spin()

# if __name__ == '__main__':
#     try:
#         node = ObstacleHandlingNode()
#         node.spin()
#     except rospy.ROSInterruptException:
#         pass
