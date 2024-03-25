import rclpy
import sys
import numpy as np
import cv2
import threading
import time
from datetime import datetime

from interfaces.motors import PublisherMotors
from interfaces.camera import ListenerCamera
from shared.image import SharedImage
from shared.value import SharedValue


IMG_WIDTH = 320
IMG_HEIGHT = 240

# ROS2 init
rclpy.create_node('HAL')

shared_image = SharedImage("halimage")
shared_v = SharedValue("velocity")
shared_w = SharedValue("angular")

# ROS2 Topics
camera = ListenerCamera("/cam_f1_left/image_raw")
motors = PublisherMotors("/cmd_vel", 4, 0.3)

start_time = 0

# Get Image from ROS Driver Camera
def getImage():
    try:
        rclpy.spin_once(camera)
        image = camera.getImage().data
        # image = self._get_test_image()
        # print(f"HAL image set, shape: {image.shape}, bytes: {image.nbytes}", flush=True)
        return image
    except Exception as e:
        print(f"Exception in hal getImage {repr(e)}")

# Set the velocity
def setV(velocity):
    motors.sendV(velocity)

# Set the angular velocity
def setW(velocity):
    motors.sendW(velocity)