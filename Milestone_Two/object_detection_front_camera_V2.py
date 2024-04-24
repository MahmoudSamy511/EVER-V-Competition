#!/usr/bin/env python3

import cv2
import cvlib as cv
from cvlib.object_detection import draw_bbox


video = cv2.VideoCapture(0)

while True:
    ret, frame = video.read()
    bbox, label, conf = cv.detect_common_objects(frame)
    output_image = draw_bbox(frame, bbox, label, conf)

    # Draw bounding box with label and confidence
    for i, box in enumerate(bbox):
        x, y, w, h = box
        label_text = f'{label[i]}: {conf[i]:.2f}'  # Format label and confidence
        cv2.rectangle(output_image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw bounding box
        cv2.putText(output_image, label_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)  # Display label and confidence

    cv2.imshow("Object Detection", output_image)

    if cv2.waitKey(1) & 0xff == ord("q"):
        break

cv2.destroyAllWindows()
video.release()
