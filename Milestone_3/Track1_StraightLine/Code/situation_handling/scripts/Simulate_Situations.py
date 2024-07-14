#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import random


# messages = [
#         "Adaptive Cruise Control",
#         "Lane Change to the right",
#         "Lane Change to the left",
#         "Emergency Stops",
#         "Slow Down",
#         "All Good",
#     ]


messages = [
        0X00,   # "Adaptive Cruise Control",
        0X01,   # "Lane Change to the right",
        0X02,   # "Lane Change to the left",
        0X03,   # "Emergency Stops",
        0X04,   # "Slow Down",
        0X05,   # "All Good",
    ]

def talker():
    pub = rospy.Publisher('situations', String, queue_size=10)
    rospy.init_node('situation_talker', anonymous=True)
    while not rospy.is_shutdown():
        situation_str = random.choice(messages)
        pub.publish("Lane Change to the left")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
