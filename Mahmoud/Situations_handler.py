#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64


class situation_handler:
    def __init__(self):
        rospy.init_node('situation_handler', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
        self.brakes_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
        self.steering_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
        rospy.Subscriber('situations', String, self.callback)
        rospy.spin()

    def Emergency_Stop(self):
        print("Emergency Stop mode")
        self.brakes_pub.publish(0.0)
        return

    def  Adaptive_Cruise_Control(self):
        print("Adaptive Cruise Control mode")
        return

    def Lane_Change(self):
        print("Lane Change mode")
        return

    def default(self):
        print("Default mode")
        return
    
    def callback(self,data):
        if data.data == "Emergency Stops":
            self.Emergency_Stop()
        elif data.data == "Adaptive Cruise Control":
            self.Adaptive_Cruise_Control()
        elif data.data == "Lane Change":
            self.Lane_Change()
        else:
            self.default()


if __name__ == '__main__':
    try:
        node = situation_handler()
    except rospy.ROSInterruptException:
        pass

