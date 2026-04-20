import rclpy
import threading
import sys

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode
from nav_msgs.msg import Odometry

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
laser = LaserNode("/f1/laser/scan")

_current_v = 0.0
_current_w = 0.0

def _odom_callback(msg):
    global _current_v, _current_w
    _current_v = msg.twist.twist.linear.x
    _current_w = msg.twist.twist.angular.z

odom_node = rclpy.create_node("odom_velocity_reader")

odom_node.create_subscription(Odometry, "/odom", _odom_callback, 10)

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(pose3d)
executor.add_node(laser)
executor.add_node(odom_node)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()

print("HAL-Nodes Thread Started")


def getPose3d():
    return pose3d.getPose3d()


def getLaserData():
    laser_data = laser.getLaserData()
    while len(laser_data.values) == 0:
        laser_data = laser.getLaserData()
    return laser_data


def setV(velocity):
    motors.sendV(float(velocity))


def setW(velocity):
    motors.sendW(float(velocity))


def getVelocity():
    """
    Returns real robot velocity from /odom:
    v -> linear x
    w -> angular z
    """
    return _current_v, _current_w


def getDynamicWindowLimits(A_V=3.0, A_W=3.0, DT=0.1, V_MAX=2.0, W_MAX=2.0):
    """
    Returns dynamic window limits based on REAL robot velocity.
    """

    v, w = getVelocity()

    v_min = max(0.0, v - A_V * DT)
    v_max = min(V_MAX, v + A_V * DT)

    w_min = max(-W_MAX, w - A_W * DT)
    w_max = min(W_MAX, w + A_W * DT)

    return v_min, v_max, w_min, w_max