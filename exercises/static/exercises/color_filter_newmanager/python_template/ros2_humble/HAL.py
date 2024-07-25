import numpy as np
import rclpy
import cv2

from jderobot_drones.drone_wrapper import DroneWrapper
from jderobot_drones.image_sub import ImageSubscriberNode

### HAL INIT ###

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init()
    

IMG_WIDTH = 320
IMG_HEIGHT = 240


# Get Image from ROS Driver Camera
def get_image():
    #image = self.cam.get_frontal_image()
    image = cv2.imread('/RoboticsAcademy/exercises/static/exercises/color_filter_newmanager/python_template/ros2_humble/image.png', cv2.IMREAD_COLOR)
    image = cv2.resize(image, (320, 240))
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image_rgb



