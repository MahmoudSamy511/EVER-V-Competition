
import rospy
from std_msgs.msg import String, Bool, Float32
import json

def multi_topic_publisher():
    rospy.init_node('multi_topic_publisher')
    
    # Publishers for each topic
    pub_detected_object_labels = rospy.Publisher('/detected_object_labels', String, queue_size=10)
    pub_is_dynamic_obstacle = rospy.Publisher('/is_dynamic_obstacle', Bool, queue_size=10)
    pub_lidar_distance = rospy.Publisher('/lidar_distance', Float32, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        # Publish to /detected_object_labels
        labels = {
            "left lane": "None",
            "center": "person",
            "right lane": "None"
        }
        msg_detected_object_labels = String()
        msg_detected_object_labels.data = json.dumps(labels)
        pub_detected_object_labels.publish(msg_detected_object_labels)
        
        # Publish to /is_dynamic_obstacle
        msg_is_dynamic_obstacle = Bool()
        msg_is_dynamic_obstacle.data = False
        pub_is_dynamic_obstacle.publish(msg_is_dynamic_obstacle)
        
        # Publish to /lidar_distance
        msg_lidar_distance = Float32()
        msg_lidar_distance.data = 0
        pub_lidar_distance.publish(msg_lidar_distance)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        multi_topic_publisher()
    except rospy.ROSInterruptException:
        pass
