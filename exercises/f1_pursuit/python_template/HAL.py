import json
import rclpy
import sys
import threading
import time

from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.camera import CameraNode

IMG_WIDTH = 320
IMG_HEIGHT = 240

freq = 90.0  # Less than this wont work


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
motor_node = MotorsNode("/f1/cmd_vel", 4, 0.3)
camera_node = CameraNode("/f1/camera/image_raw")

_race_state = ["racing"]


def _status_callback(msg):
    try:
        _race_state[0] = json.loads(msg.data)["state"]
    except Exception:
        pass


# The rival referees the race and latches how it ended
status_node = rclpy.create_node("hal_pursuit_status")
status_node.create_subscription(
    String,
    "/f1_pursuit/status",
    _status_callback,
    QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
)

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(camera_node)
executor.add_node(status_node)
executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()


# Get Image from ROS Driver Camera
def getImage():
    image = camera_node.getImage()
    while image is None:
        image = camera_node.getImage()
    return image.data


# True once the rival has been caught
def isCaught():
    return _race_state[0] == "caught"


# The race is over once the rival is caught or gets away, so the car stops
def _race_over():
    if _race_state[0] == "racing":
        return False
    motor_node.sendV(0.0)
    motor_node.sendW(0.0)
    return True


# Set the velocity
def setV(velocity):
    if _race_over():
        return
    motor_node.sendV(float(velocity))


# Set the angular velocity
def setW(velocity):
    if _race_over():
        return
    motor_node.sendW(float(velocity))
