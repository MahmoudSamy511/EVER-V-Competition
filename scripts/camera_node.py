#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cvlib as cv
from cvlib.object_detection import draw_bbox

class ObjectDetectionSubscriber:
    def __init__(self):
        rospy.init_node('object_detection_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image", Image, self.image_callback)
        self.window_name = 'Object Detection'
        self.color_green = (0, 255, 0)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            bbox, label, conf = cv.detect_common_objects(cv_image)
            output_image = draw_bbox(cv_image, bbox, label, conf)
            cv2.imshow(self.window_name, output_image)
            cv2.waitKey(1)
        except Exception as e:
            print(e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ObjectDetectionSubscriber()
        node.run()
    except rospy.ROSInterruptException:
        pass
