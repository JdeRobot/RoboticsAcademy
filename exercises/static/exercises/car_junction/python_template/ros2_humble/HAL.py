import rclpy
import threading
import time
import sys

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.camera import CameraNode
from hal_interfaces.specific.car_junction.lidar import LidarNode

# Hardware Abstraction Layer
IMG_WIDTH = 320
IMG_HEIGHT = 240
freq = 30.0

# Mutes exceptions
def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook

# ROS2 init
print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

### HAL INIT ###
motor_node = MotorsNode("/cmd_vel", 40, 0)
odometry_node = OdometryNode("/odom")
camera_front_node = CameraNode("/waymo/camera_front")
lidar_node = LidarNode("/waymo/lidar/points")


# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(odometry_node)
executor.add_node(motor_node)
executor.add_node(camera_front_node)
executor.add_node(lidar_node)

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


def getFrontCameraData():
    image = camera_front_node.getImage()
    while image == None:
        image = camera_front_node.getImage()
    return image.data

def getLidarData():
    lidar = lidar_node.getLidarData()
    timestamp = lidar.timeStamp
    while timestamp == 0.0:
        lidar = lidar_node.getLidarData()
        timestamp = lidar.timeStamp
    return lidar

def setV(velocity):
    motor_node.sendV(float(velocity))