#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import random


messages = [
        "Adaptive Cruise Control",
        "Lane Change to the right",
        "Lane Change to the left",
        "Emergency Stops",
        "Gradually Stops",
        "All Good",
    ]

def talker():
    rospy.init_node('situation_talker', anonymous=True)
    pub = rospy.Publisher('situations', String, queue_size=10)
    
    while not rospy.is_shutdown():
        situation_str = random.choice(messages)
        pub.publish("Adaptive Cruise Control")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
