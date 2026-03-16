from pathlib import Path

TARGET = Path("/RoboticsAcademy/filesystem/follow_line/academy.py")
TARGET.parent.mkdir(parents=True, exist_ok=True)

BASELINE = """# pylint: disable=all
import time

import cv2
import numpy as np

import HAL
import WebGUI as GUI

# Red line in HSV is usually split around 0 and 180.
LOW_RED_1 = np.array([0, 60, 60], dtype=np.uint8)
HIGH_RED_1 = np.array([12, 255, 255], dtype=np.uint8)
LOW_RED_2 = np.array([168, 60, 60], dtype=np.uint8)
HIGH_RED_2 = np.array([180, 255, 255], dtype=np.uint8)

# Conservative defaults to favor stability over max speed.
KP = 0.006
KD = 0.0018
BASE_SPEED = 2.8
MIN_SPEED = 1.0
MAX_SPEED = 3.2
MAX_W = 0.9
ERR_SMOOTH_ALPHA = 0.7

prev_error = 0.0
prev_time = time.time()
last_turn_sign = 1.0

while True:
    image = HAL.getImage()
    if image is None:
        HAL.setV(0.0)
        HAL.setW(0.0)
        time.sleep(0.03)
        continue

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, LOW_RED_1, HIGH_RED_1)
    mask2 = cv2.inRange(hsv, LOW_RED_2, HIGH_RED_2)
    mask = cv2.bitwise_or(mask1, mask2)

    h, w = mask.shape
    roi_y = int(h * 0.55)
    roi = mask[roi_y:, :]
    moments = cv2.moments(roi)

    if moments["m00"] > 0:
        cx = int(moments["m10"] / moments["m00"])
        raw_error = (w * 0.5) - cx

        now = time.time()
        dt = max(now - prev_time, 1e-3)
        error = (ERR_SMOOTH_ALPHA * prev_error) + ((1.0 - ERR_SMOOTH_ALPHA) * raw_error)
        d_error = (error - prev_error) / dt
        w_cmd = (KP * error) + (KD * d_error)
        w_cmd = float(np.clip(w_cmd, -MAX_W, MAX_W))

        v_cmd = BASE_SPEED - (0.015 * min(abs(error), w * 0.5))
        v_cmd = float(np.clip(v_cmd, MIN_SPEED, MAX_SPEED))

        HAL.setV(v_cmd)
        HAL.setW(w_cmd)

        last_turn_sign = 1.0 if error >= 0 else -1.0
        prev_error = error
        prev_time = now

        cv2.circle(image, (cx, roi_y + 20), 8, (0, 255, 0), -1)
    else:
        # Lost line: slow down and search in last known direction.
        HAL.setV(0.5)
        HAL.setW(0.45 * last_turn_sign)

    GUI.showImage(image)
    time.sleep(0.03)
"""

TARGET.write_text(BASELINE, encoding="utf-8")
print("baseline written:", TARGET)
