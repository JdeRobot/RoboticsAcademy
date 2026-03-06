import rclpy
import threading
import time
import sys

from std_msgs.msg import Float64

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode
from hal_interfaces.general.sim_time import SimTimeNode

# Hardware Abstraction Layer
freq = 30.0

# Lift State
liftState = False

# Mutes exceptions
def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook

print("HAL 2 (HARMONIC) initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

### HAL INIT ###
motor_node = MotorsNode("/amazon_robot/cmd_vel", 4, 0.3)
odometry_node = OdometryNode("/amazon_robot/odom")
laser_node = LaserNode("/amazon_robot/scan")
sim_time_node = SimTimeNode()

# Platform control (Harmonic direct topic)
platform_node = rclpy.create_node("platform_cmd_node")
platform_pub = platform_node.create_publisher(Float64, "/platform/cmd_vel", 10)

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(odometry_node)
executor.add_node(laser_node)
executor.add_node(sim_time_node)
executor.add_node(platform_node)


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


def getSimTime():
    return sim_time_node.getSimTime()


def getLaserData():
    laser_data = laser_node.getLaserData()
    while len(laser_data.values) == 0:
        laser_data = laser_node.getLaserData()
    return laser_data


def setV(velocity):
    motor_node.sendV(float(velocity))


def setW(velocity):
    motor_node.sendW(float(velocity))


def lift():
    global liftState
    liftState = True
    msg = Float64()
    msg.data = 5.0
    platform_pub.publish(msg)


def putdown():
    global liftState
    liftState = False
    msg = Float64()
    msg.data = -5.0
    platform_pub.publish(msg)


def getLiftState():
    global liftState
    return liftState