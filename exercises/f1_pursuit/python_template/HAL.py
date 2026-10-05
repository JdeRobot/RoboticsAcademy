import math
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Bool

from hal_interfaces.general.camera import CameraNode
from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode

IMG_WIDTH = 320
IMG_HEIGHT = 240

freq = 90.0 

CAR_NAMESPACE = "f1"
RIVAL_NAMESPACE = "f1_rival"

CATCH_RADIUS = 2.5


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


# ROS2 init
if not rclpy.ok():
    rclpy.init(args=sys.argv)

# ROS2 Topics
motor_node = MotorsNode("/" + CAR_NAMESPACE + "/cmd_vel", 4, 0.3)
camera_node = CameraNode("/" + CAR_NAMESPACE + "/camera/image_raw")
odom_node = OdometryNode("/" + CAR_NAMESPACE + "/odom", "hal_odom")
rival_odom_node = OdometryNode("/" + RIVAL_NAMESPACE + "/odom", "hal_rival_odom")

_armed = [False]


def _armed_callback(msg):
    _armed[0] = msg.data


# The rival publishes this once it has pulled clear of the grid
armed_node = rclpy.create_node("hal_armed_node")
armed_node.create_subscription(
    Bool,
    "/f1_pursuit/armed",
    _armed_callback,
    QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
)

# Spin nodes so that subscription callbacks load topic data
executor = MultiThreadedExecutor()
executor.add_node(camera_node)
executor.add_node(odom_node)
executor.add_node(rival_odom_node)
executor.add_node(armed_node)
executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()


### GETTERS ###


# Get Image from ROS Driver Camera
def getImage():
    image = camera_node.getImage()
    while image is None:
        image = camera_node.getImage()
    return image.data


def getPose3d():
    return odom_node.getPose3d()


def getRivalPose3d():
    return rival_odom_node.getPose3d()


def getRivalPosition():
    pose = rival_odom_node.getPose3d()
    return [pose.x, pose.y, pose.z]


# Distance to the rival in the ground plane
def getGap():
    here = odom_node.getPose3d()
    there = rival_odom_node.getPose3d()
    return math.hypot(here.x - there.x, here.y - there.y)


def isRacing():
    return _armed[0]


def isCaught():
    if not _armed[0]:
        return False
    rival = rival_odom_node.getPose3d()
    if not any(abs(v) > 1e-6 for v in (rival.x, rival.y, rival.z)):
        return False
    return getGap() < CATCH_RADIUS


### SETTERS ###

_stopped = False


# The rival stops once it is caught, so the chaser stops too
def _stop():
    global _stopped
    if not isCaught():
        return False
    if not _stopped:
        _stopped = True
        motor_node.sendV(0.0)
        motor_node.sendW(0.0)
    return True


# Set the velocity
def setV(velocity):
    if _stop():
        return
    motor_node.sendV(float(velocity))


# Set the angular velocity
def setW(velocity):
    if _stop():
        return
    motor_node.sendW(float(velocity))
