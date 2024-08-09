import GUI
import HAL
import base64
from datetime import datetime
import os
import sys
import cv2
import numpy as np
import onnxruntime as rt

raw_dl_model = '/workspace/code/demo_model/Demo_Model.onnx'

def display_output_detection(img, detections, scores):
    """Draw box and label for the detections."""
    height, width = img.shape[:2]
    for i, detection in enumerate(detections):
        top = int(max(0, np.floor(detection[0] * height + 0.5)))
        left = int(max(0, np.floor(detection[1] * width + 0.5)))
        bottom = int(min(height, np.floor(detection[2] * height + 0.5)))
        right = int(min(width, np.floor(detection[3] * width + 0.5)))
        cv2.rectangle(img, (left, top), (right, bottom), (0, 0, 255), 2)
        cv2.putText(img, 'Human', (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)

def display_gt_detection(img, gt_detections):
    """Draw ground truth bounding boxes."""
    for gt_detection in gt_detections:
        left, top, right, bottom = map(int, gt_detection[1:5])
        cv2.rectangle(img, (left, top), (right, bottom), (0, 255, 0), 2)

# Load ONNX model
sess = rt.InferenceSession(raw_dl_model)
input_layer_name = sess.get_inputs()[0].name
output_layers_names = [output.name for output in sess.get_outputs()]

while True:
    # Get input webcam image
    success, img = HAL.getVid()
    if not success:
        break

    img_resized = cv2.resize(img, (300, 300))
    img_data = np.reshape(
        img_resized, (1, img_resized.shape[0], img_resized.shape[1], img_resized.shape[2]))

    # Inference
    result = sess.run(output_layers_names, {input_layer_name: img_data})
    detection_boxes, detection_classes, detection_scores, num_detections = result
    count = 0
    detections = []
    scores = []
    batch_size = num_detections.shape[0]

    for batch in range(0, batch_size):
        for detection in range(0, int(num_detections[batch])):
            c = detection_classes[batch][detection]
            # Skip if not human class
            if c != 1:
                GUI.showImage(img)
                continue

            count += 1
            d = detection_boxes[batch][detection]
            detections.append(d)
            score = detection_scores[batch][detection]
            scores.append(score)
    
    display_output_detection(img, detections, scores)
    HAL.frame_number += 1

    # To print FPS after each frame has been fully processed and displayed
    GUI.showImage(img)
    

