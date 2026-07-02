import rclpy
import threading
import time
import sys
import subprocess
import re

from hal_interfaces.specific.o3de_follow_line.acker_motors import AckerMotorsNode
from hal_interfaces.general.camera import CameraNode


# Hardware Abstraction Layer
freq = 120.0

IMG_WIDTH = 320
IMG_HEIGHT = 240

# Mutes exceptions
def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook


def __auto_spin() -> None:
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0)
        except Exception:
            pass
        time.sleep(1 / freq)


print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

### HAL INIT ###
motor_node = AckerMotorsNode("/cmd_vel", 4, 0.3, 2)
camera_node = CameraNode("/cam_f1_left/image_raw")

executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(camera_node)
executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()
    
def getImage():
    image = camera_node.getImage()
    while image == None:
        image = camera_node.getImage()
    return image.data

def setV(velocity):
    motor_node.setV(float(velocity));

def setW(angular_velocity):
    motor_node.setW(float(angular_velocity));

print("HAL ready — publishing /cmd_vel continuously", flush=True)
    
    
