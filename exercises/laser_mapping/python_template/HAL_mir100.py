import rclpy
import sys
import threading
import time

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode

# Hardware Abstraction Layer for the MiR100, same speed limits as in amazon_warehouse
freq = 90.0

MAX_V = 1.0
MAX_W = 1.0


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


print("HAL MiR100 initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=sys.argv)

motor_node = MotorsNode("/do150/cmd_vel", MAX_V, MAX_W)
odometry_node = OdometryNode("/do150/odom")
noisy_odometry_node = OdometryNode(
    "/do150/odom_noisy", node_name="noisy_odometry_node"
)
front_laser_node = LaserNode("/do150/front_laser/scan")
back_laser_node = LaserNode("/do150/back_laser/scan")

executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(odometry_node)
executor.add_node(noisy_odometry_node)
executor.add_node(front_laser_node)
executor.add_node(back_laser_node)

executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()


def getPose3d():
    return odometry_node.getPose3d()


def getOdom():
    return noisy_odometry_node.getPose3d()


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


def setV(v):
    motor_node.sendV(max(-MAX_V, min(MAX_V, float(v))))


def setW(w):
    motor_node.sendW(max(-MAX_W, min(MAX_W, float(w))))
