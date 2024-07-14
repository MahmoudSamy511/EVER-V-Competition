#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
import tf.transformations
import math

class LaneChanger:
    def __init__(self):
        rospy.init_node('lane_changer', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
        self.brakes_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
        self.steering_angle_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
        self.start_pub = rospy.Publisher('/startSimulation', Bool, queue_size=10)
        

        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.current_position = None
        self.target_lane_center = 0.0  # Center of the lane
        self.lane_width = 3.0
        self.max_steering_angle = 15.0  # Maximum steering angle in degrees
        self.change_in_progress = False
        self.orientation_q = 0.0
        self.current_orientation = 0.0

    def odom_callback(self, data):
        self.current_position = data.pose.pose.position
        self.orientation_q = data.pose.pose.orientation

        orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.current_orientation = math.degrees(yaw)

    def change_lane(self, direction):
        rospy.loginfo(f'direction is {direction}')
        self.target_lane_center = self.current_position.y
        self.steering_angle_pub.publish(0.0)
        
        if self.current_position is None:
            return

        if direction == 'left':
            self.target_lane_center -= self.lane_width
        elif direction == 'right':
            self.target_lane_center += self.lane_width

        self.change_in_progress = True

        while not rospy.is_shutdown() and self.change_in_progress:
            if self.current_position is None:
                rospy.loginfo("Waiting for position data...")
                continue

            error = abs(self.target_lane_center) - abs(self.current_position.y)

            steering_angle = max(min(error * 5.0, self.max_steering_angle), -self.max_steering_angle)
            rospy.loginfo("Current position: y = %.2f, Target position: y = %.2f, Steering angle: %.2f" %
                          (self.current_position.y, self.target_lane_center, steering_angle))

            if abs(error) < 0.2:  # If close enough to the target
                self.change_in_progress = False
                rospy.loginfo("Lane change completed.")
                steering_angle = 0.0

            # Publish commands
            # if direction == 'right':
            #     self.steering_angle_pub.publish(Float64(-1 * steering_angle))
            # else:
            #     self.steering_angle_pub.publish(Float64(-steering_angle))
            
            self.steering_angle_pub.publish(Float64(-steering_angle))
            self.cmd_vel_pub.publish(Float64(0.1))
            
        
        if direction == 'right':
            while self.current_orientation < 78.43:
                self.steering_angle_pub.publish(Float64(steering_angle))
                steering_angle += 1
        else:
            while self.current_orientation > 100.0:
                print(f'steering angle {steering_angle}')
                steering_angle = max(steering_angle, -self.max_steering_angle)
                self.steering_angle_pub.publish(Float64(steering_angle))
                steering_angle -= 1



        for _ in range(4):
            if direction == 'right':
                self.steering_angle_pub.publish(Float64(-1 * steering_angle))
            else:
                self.steering_angle_pub.publish(Float64(steering_angle))
        
        self.steering_angle_pub.publish(0.0)

if __name__ == '__main__':
    try:
        lane_changer = LaneChanger()
        # lane_changer.start_pub.publish(Bool(True))
        rospy.sleep(1)
        
        # Move forward a bit before changing lanes
        lane_changer.brakes_pub.publish(0.0)
        lane_changer.cmd_vel_pub.publish(Float64(0.5))
        rospy.sleep(5)
        
        lane_changer.change_lane('left')
    
        lane_changer.cmd_vel_pub.publish(Float64(0.5))
        rospy.sleep(5)
        
        lane_changer.change_lane('right')        
        
        lane_changer.cmd_vel_pub.publish(Float64(0.5))
        rospy.sleep(5)
        
        # Stop the car once the lane change is complete
        rospy.loginfo("Stopping the car.")
        lane_changer.cmd_vel_pub.publish(Float64(0.0))
        lane_changer.brakes_pub.publish(Float64(1.0))

        rospy.spin()
        
        
    except rospy.ROSInterruptException:
        pass
