import rclpy
import threading
import sys

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode

# Hardware Abstraction Layer
IMG_WIDTH = 320
IMG_HEIGHT = 240


# Mutes exceptions
def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook

# ROS2 init
if not rclpy.ok():
    rclpy.init(args=None)
    rclpy.create_node("HAL")

pose3d = OdometryNode("/odom")
motors = MotorsNode("/cmd_vel", 4, 0.3)

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(pose3d)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()


# Pose
def getPose3d():
    return pose3d.getPose3d()


def setV(velocity):
    motors.sendV(float(velocity))


def setW(velocity):
    motors.sendW(float(velocity))
