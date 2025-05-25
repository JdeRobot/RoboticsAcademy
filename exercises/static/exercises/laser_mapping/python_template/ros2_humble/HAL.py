import rclpy
import sys
import threading
import time
import numpy as np

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from real_noise_odometry import NoisyOdometryNode
from hal_interfaces.general.laser import LaserNode

freq = 90.0

def __auto_spin() -> None:
    while rclpy.ok():
        executor.spin_once(timeout_sec=0)
        time.sleep(1/freq)


if not rclpy.ok():
    rclpy.init(args=sys.argv)

### HAL INIT ###
motor_node = MotorsNode("/turtlebot3/cmd_vel", 4, 0.3)
odometry_node = OdometryNode("/turtlebot3/odom")
noisy_odometry_node = NoisyOdometryNode("/turtlebot3/odom")
noisy_odometry_node.noise_level = 0.001
noisy_odometry_node_2 = NoisyOdometryNode("/turtlebot3/odom")
noisy_odometry_node_2.noise_level = 0.03
noisy_odometry_node_3 = NoisyOdometryNode("/turtlebot3/odom")
noisy_odometry_node_3.noise_level = 0.06
laser_node = LaserNode("/turtlebot3/laser/scan")

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(odometry_node)
executor.add_node(noisy_odometry_node)
executor.add_node(noisy_odometry_node_2)
executor.add_node(noisy_odometry_node_3)
executor.add_node(laser_node)

executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()


### GETTERS ### 

# Pose
def getPose3d():
    try:
        return odometry_node.getPose3d()
    except Exception as e:
        print(f"Exception in hal getPose3d {repr(e)}")  

# Pose
def getOdom():
    try:
        return noisy_odometry_node.getPose3d()
    except Exception as e:
        print(f"Exception in hal getPose3d {repr(e)}")  

def getOdom2():
    try:
        return noisy_odometry_node_2.getPose3d()
    except Exception as e:
        print(f"Exception in hal getPose3d {repr(e)}")

def getOdom3():
    try:
        return noisy_odometry_node_3.getPose3d()
    except Exception as e:
        print(f"Exception in hal getPose3d {repr(e)}")

def getLaserData():
    laser_data = laser_node.getLaserData()
    while len(laser_data.values) == 0:
        laser_data = laser_node.getLaserData()
    return laser_data


### SETTERS ###

# Linear speed
def setV(v):
    motor_node.sendV(float(v))

# Angular speed
def setW(w):
    motor_node.sendW(float(w))