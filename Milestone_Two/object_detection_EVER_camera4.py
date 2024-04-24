#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cvlib as cv
from cvlib.object_detection import draw_bbox

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image", Image, self.image_callback)
        self.window_name = 'Object Detection'

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.detect_objects(cv_image)
        except Exception as e:
            print(e)

    def detect_objects(self, cv_image):
        bbox, label, conf = cv.detect_common_objects(cv_image)
        output_image = draw_bbox(cv_image, bbox, label, conf)

        # Draw bounding box with label and confidence
        # for i, box in enumerate(bbox):
            # x, y, w, h = box
            # label_text = f'{label[i]}: {conf[i]:.2f}'  # Format label and confidence
            # cv2.rectangle(output_image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw bounding box
            # cv2.putText(output_image, label_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)  # Display label and confidence

        cv2.imshow(self.window_name, output_image)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_subscriber = ImageSubscriber()
        image_subscriber.run()
    except rospy.ROSInterruptException:
        pass
