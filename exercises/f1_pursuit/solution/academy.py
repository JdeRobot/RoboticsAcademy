import cv2
import numpy as np

import HAL
import WebGUI  # noqa: F401  starts the camera panels and the console
from Frequency import tick

ROWS = (0.88, 0.80, 0.72, 0.63, 0.54)
MIN_RUN = 3

KP = 1.35
KD = 0.055
K_CURVE = 0.9
W_CLAMP = 1.6

V_STRAIGHT = 4.2
V_CORNER = 1.3
CORNER_BRAKE = 1.5

track_x = None
last_error = 0.0


def red_mask(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Red wraps the hue origin, so it takes two ranges
    return cv2.inRange(hsv, (0, 110, 70), (10, 255, 255)) | cv2.inRange(
        hsv, (170, 110, 70), (180, 255, 255)
    )


def row_runs(row):
    hits = np.flatnonzero(row)
    if hits.size < MIN_RUN:
        return []
    splits = np.flatnonzero(np.diff(hits) > 1)
    return [(r.mean(), r.size) for r in np.split(hits, splits + 1) if r.size >= MIN_RUN]


def follow_line(mask, seed):
    """Sample the line bottom to top, each row anchored on the one below it.

    Both lanes are painted the same red, so continuity is what separates this
    car's line from the rival's.
    """
    h, w = mask.shape[:2]
    columns = []
    anchor = seed
    for frac in ROWS:
        runs = row_runs(mask[int(h * frac), :])
        if not runs:
            columns.append(None)
            continue
        best = min(runs, key=lambda r: abs(r[0] - anchor))
        if columns and columns[0] is not None and abs(best[0] - anchor) > w * 0.45:
            columns.append(None)
            continue
        anchor = best[0]
        columns.append(best[0])
    return columns


while True:
    image = HAL.getImage()
    h, w = image.shape[:2]

    mask = red_mask(image)
    if track_x is None:
        track_x = w / 2.0

    columns = follow_line(mask, track_x)
    near = columns[0]

    if near is None:
        HAL.setV(0.7)
        HAL.setW(0.7 if track_x < w / 2.0 else -0.7)
    else:
        track_x = near
        error = (near - w / 2.0) / (w / 2.0)
        far = next((c for c in reversed(columns) if c is not None), near)
        curve = (far - near) / (w / 2.0)

        steer = -(KP * error + KD * (error - last_error) + K_CURVE * curve)
        steer = max(-W_CLAMP, min(W_CLAMP, steer))
        last_error = error

        bend = min(1.0, abs(curve) * CORNER_BRAKE + 0.4 * abs(error))
        speed = V_STRAIGHT - (V_STRAIGHT - V_CORNER) * bend

        HAL.setV(max(V_CORNER, speed))
        HAL.setW(steer)

    if HAL.isCaught():
        print("caught the rival", flush=True)

    tick(50)
