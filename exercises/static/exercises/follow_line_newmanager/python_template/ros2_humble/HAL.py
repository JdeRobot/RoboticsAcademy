import rclpy
import sys
import threading
import time

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.camera import CameraNode


IMG_WIDTH = 320
IMG_HEIGHT = 240

freq = 35.0

# def __auto_spin() -> None:
#     while rclpy.ok():
#         executor.spin_once(timeout_sec=0)
#         time.sleep(1/freq)

# ROS2 init
if not rclpy.ok():
    rclpy.init(args=sys.argv)

# ROS2 Topics
motor_node = MotorsNode("/cmd_vel", 4, 0.3)
camera_node = CameraNode("/cam_f1_left/image_raw")

# Spin nodes so that subscription callbacks load topic data
# executor = rclpy.executors.MultiThreadedExecutor()
# executor.add_node(camera_node)
# executor_thread = threading.Thread(target=executor.spin, daemon=True)
# executor_thread.start()

# Get Image from ROS Driver Camera
# def getImage():
#     return camera_node.getImage().data

def getImage():
    try:
        rclpy.spin_once(camera_node)
        image = camera_node.getImage().data
        return image
    except Exception as e:
        print(f"Exception in hal getImage {repr(e)}")


# Set the velocity
def setV(velocity):
    motor_node.sendV(float(velocity))

# Set the angular velocity
def setW(velocity):
    motor_node.sendW(float(velocity))