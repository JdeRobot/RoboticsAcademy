import GUI
import HAL
import base64
from datetime import datetime
import json
import sys
import time
import cv2
import numpy as np
import onnxruntime

roi_scale = 0.75
input_size = (28, 28)

# Receive model
raw_dl_model = '/workspace/code/demo_model/mnist_cnn.onnx'

# Load ONNX model
try:
    ort_session = onnxruntime.InferenceSession(raw_dl_model)
except Exception:
    exc_type, exc_value, exc_traceback = sys.exc_info()
    print(str(exc_value))
    print("ERROR: Model couldn't be loaded")

previous_pred = 0
previous_established_pred = "-"
count_same_digit = 0

while True:

    # Get input webcam image
    image = HAL.getImage()
    if image is not None:
        input_image_gray = np.mean(image, axis=2).astype(np.uint8)

        # Get original image and ROI dimensions
        h_in, w_in = image.shape[:2]
        min_dim_in = min(h_in, w_in)
        h_roi, w_roi = (int(min_dim_in * roi_scale), int(min_dim_in * roi_scale))
        h_border, w_border = (int((h_in - h_roi) / 2.), int((w_in - w_roi) / 2.))

        # Extract ROI and convert to tensor format required by the model
        roi = input_image_gray[h_border:h_border + h_roi, w_border:w_border + w_roi]
        roi_norm = (roi - np.mean(roi)) / np.std(roi)
        roi_resized = cv2.resize(roi_norm, input_size)
        input_tensor = roi_resized.reshape((1, 1, input_size[0], input_size[1])).astype(np.float32)

        # Inference
        ort_inputs = {ort_session.get_inputs()[0].name: input_tensor}
        output = ort_session.run(None, ort_inputs)[0]
        pred = int(np.argmax(output, axis=1))  # get the index of the max log-probability

        # Show region used as ROI
        cv2.rectangle(image, pt2=(w_border, h_border), pt1=(w_border + w_roi, h_border + h_roi), color=(255, 0, 0), thickness=3)

        # Show FPS count
        cv2.putText(image, "Pred: {}".format(int(pred)), (7, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Send result
        GUI.showImage(image)
   

