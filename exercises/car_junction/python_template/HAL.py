import rclpy
import threading
import time
import sys

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.camera import CameraNode

# Hardware Abstraction Layer
freq = 30.0


# Mutes exceptions
def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook


print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

motor_node = MotorsNode("/cmd_vel", 40, 0)
odometry_node = OdometryNode("/odom")
camera_node = CameraNode("/prius_autoparking/image_raw")


# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(odometry_node)
executor.add_node(motor_node)
executor.add_node(camera_node)


def __auto_spin() -> None:
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0)
        except Exception:
            pass
        time.sleep(1 / freq)


executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()


def getPose3d():
    return odometry_node.getPose3d()


def getImage():
    image = camera_node.getImage()
    while image is None:
        image = camera_node.getImage()
    return image.data


def setV(velocity):
    motor_node.sendV(float(velocity))
