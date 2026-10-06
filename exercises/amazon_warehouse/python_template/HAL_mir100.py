import rclpy
import threading
import time
import sys

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode

# Hardware Abstraction Layer for the MiR100, the topics are the same for the
# simulated robot and for the real one through mir100_bridge
freq = 30.0

# Speed limits, they protect the real robot from a wrong command
MAX_V = 1.0
MAX_W = 1.0


# Mutes exceptions
def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook

print("HAL MiR100 initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

### HAL INIT ###
motor_node = MotorsNode("/mir100/cmd_vel", MAX_V, MAX_W)
odometry_node = OdometryNode("/mir100/odom")
front_laser_node = LaserNode("/mir100/front_laser/scan")
back_laser_node = LaserNode("/mir100/back_laser/scan")

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(odometry_node)
executor.add_node(front_laser_node)
executor.add_node(back_laser_node)


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


def getFrontLaserData():
    laser_data = front_laser_node.getLaserData()
    while len(laser_data.values) == 0:
        laser_data = front_laser_node.getLaserData()
    return laser_data


def getBackLaserData():
    laser_data = back_laser_node.getLaserData()
    while len(laser_data.values) == 0:
        laser_data = back_laser_node.getLaserData()
    return laser_data


def setV(velocity):
    motor_node.sendV(max(-MAX_V, min(MAX_V, float(velocity))))


def setW(velocity):
    motor_node.sendW(max(-MAX_W, min(MAX_W, float(velocity))))
