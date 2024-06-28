#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import random


messages = [
        "Adaptive Cruise Control",
        "Lane Change",
        "Emergency Stops",
        "All Good",
    ]

def talker():
    pub = rospy.Publisher('situations', String, queue_size=10)
    rospy.init_node('situation_talker', anonymous=True)
    while not rospy.is_shutdown():
        situation_str = random.choice(messages)
        pub.publish(situation_str)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
