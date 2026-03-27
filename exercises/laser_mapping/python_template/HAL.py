import rclpy
import sys
import threading
import time

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode

freq = 90.0


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


if not rclpy.ok():
    rclpy.init(args=sys.argv)

motor_node = MotorsNode("/turtlebot3/cmd_vel", 4, 0.3)
odometry_node = OdometryNode("/turtlebot3/odom")
noisy_odometry_node = OdometryNode(
    "/turtlebot3/odom_noisy", node_name="noisy_odometry_node"
)
laser_node = LaserNode("/turtlebot3/laser/scan")

executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(odometry_node)
executor.add_node(noisy_odometry_node)
executor.add_node(laser_node)

executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()


def getPose3d():
    return odometry_node.getPose3d()


def getOdom():
    return noisy_odometry_node.getPose3d()


def getLaserData():
    laser_data = laser_node.getLaserData()
    while len(laser_data.values) == 0:
        laser_data = laser_node.getLaserData()
    return laser_data


def setV(v):
    motor_node.sendV(float(v))


def setW(w):
    motor_node.sendW(float(w))
