import rclpy
import sys
import numpy as np  # Unused, consider removing if not needed later
import cv2  # Unused, consider removing if not needed later
import threading  # Unused, consider removing if not needed later
import time  # Unused, consider removing if not needed later
from datetime import datetime  # Unused, consider removing if not needed later

from interfaces.camera import ListenerCamera
from shared.image import SharedImage

IMG_WIDTH = 320
IMG_HEIGHT = 240


# Initialize ROS2
rclpy.init(args=sys.argv)

    # Create a node
node = rclpy.create_node('HAL')

shared_image = SharedImage("halimage")

    # ROS2 Topics
camera = ListenerCamera("/v4l2_camera_node/image_raw")

    # Get Image from ROS Driver Camera
def getImage():
    try:
        rclpy.spin_once(camera)
        image = camera.getImage().data
            # print(f"HAL image set, shape: {image.shape}, bytes: {image.nbytes}", flush=True)
        return image
    except Exception as e:
        print(f"Exception in hal getImage {repr(e)}")

    # Example usage
image = getImage()
if image is not None:
    print(f"Image received with shape: {image.shape}")

    # Shutdown ROS2
rclpy.shutdown()

