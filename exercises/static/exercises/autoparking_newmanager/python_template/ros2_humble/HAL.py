import rclpy
import threading
import time

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode

# Hardware Abstraction Layer
IMG_WIDTH = 320
IMG_HEIGHT = 240
freq = 30.0

# ROS2 init

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

    ### HAL INIT ###
    motor_node = MotorsNode("/prius_autoparking/cmd_vel", 4, 0.3)
    odometry_node = OdometryNode("/prius_autoparking/odom")
    laser_front_node = LaserNode("/prius_autoparking/scan_front")
    laser_right_node = LaserNode("/prius_autoparking/scan_side")
    laser_back_node = LaserNode("/prius_autoparking/scan_back")

    # Spin nodes so that subscription callbacks load topic data
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(odometry_node)
    executor.add_node(laser_front_node)
    executor.add_node(laser_right_node)
    executor.add_node(laser_back_node)
    def __auto_spin() -> None:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0)
            time.sleep(1/freq)
    executor_thread = threading.Thread(target=__auto_spin, daemon=True)
    executor_thread.start()


def getPose3d():
    return odometry_node.getPose3d()

def getFrontLaserData():
    laser = laser_front_node.getLaserData()
    timestamp = laser.timeStamp
    while timestamp == 0.0:
        laser = laser_front_node.getLaserData()
        timestamp = laser.timeStamp
    return laser

def getRightLaserData():
    laser = laser_right_node.getLaserData()
    timestamp = laser.timeStamp
    while timestamp == 0.0:
        laser = laser_right_node.getLaserData()
        timestamp = laser.timeStamp
    return laser

def getBackLaserData():
    laser = laser_back_node.getLaserData()
    timestamp = laser.timeStamp
    while timestamp == 0.0:
        laser = laser_back_node.getLaserData()
        timestamp = laser.timeStamp
    return laser

def setV(velocity):
    motor_node.sendV(float(velocity))

def setW(velocity):
    motor_node.sendW(float(velocity))