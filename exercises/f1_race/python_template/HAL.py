import json
import rclpy
import threading
import sys

from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode

# Hardware Abstraction Layer
IMG_WIDTH = 320
IMG_HEIGHT = 240
# {"name": "target10", "x": -47, "y": 45},


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

pose3d = OdometryNode("/f1/odom")
motors = MotorsNode("/f1/cmd_vel", 4, 0.3)
laser = LaserNode("/f1/laser/scan")

_race_state = ["racing"]


def _status_callback(msg):
    try:
        _race_state[0] = json.loads(msg.data)["state"]
    except Exception:
        pass


# The rival referees the race and latches how it ended
status_node = rclpy.create_node("hal_race_status")
status_node.create_subscription(
    String,
    "/f1_race/status",
    _status_callback,
    QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
)

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(pose3d)
executor.add_node(laser)
executor.add_node(status_node)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()


def getPose3d():
    return pose3d.getPose3d()


def getLaserData():
    laser_data = laser.getLaserData()
    while len(laser_data.values) == 0:
        laser_data = laser.getLaserData()
    return laser_data


# True once the student finishes the lap before the rival
def hasWon():
    return _race_state[0] == "won"


# The race is over once either car finishes the lap, so the car stops
def _race_over():
    if _race_state[0] == "racing":
        return False
    motors.sendV(0.0)
    motors.sendW(0.0)
    return True


def setV(velocity):
    if _race_over():
        return
    motors.sendV(float(velocity))


def setW(velocity):
    if _race_over():
        return
    motors.sendW(float(velocity))
