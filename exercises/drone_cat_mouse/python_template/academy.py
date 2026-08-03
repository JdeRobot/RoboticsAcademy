import HAL
import WebGUI
import cv2
import numpy as np
import time

# You are the cat. Chase the magenta mouse drone using your camera.
#
#   HAL.get_frontal_image()            the picture
#   HAL.set_cmd_vel(vx, vy, vz, az)    speeds in the drone's own frame
#   HAL.takeoff(h), HAL.land()
#   HAL.get_mouse_position()           the truth, for checking your tracker
#   WebGUI.showImage(img), WebGUI.showLeftImage(img)

HAL.takeoff(3.0)

while True:
    image = HAL.get_frontal_image()

    HAL.set_cmd_vel(0.0, 0.0, 0.0, 0.0)

    WebGUI.showImage(image)
    time.sleep(0.05)
