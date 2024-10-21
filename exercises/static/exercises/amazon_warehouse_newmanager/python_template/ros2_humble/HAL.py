import rclpy
import threading
import time

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode
from hal_interfaces.general.sim_time import SimTimeNode
from hal_interfaces.specific.amazon_warehouse.platform_controller import PlatformCommandNode, PublisherPlatformNode

# Hardware Abstraction Layer
freq = 30.0

# Lift State
liftState = False

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

    ### HAL INIT ###
    motor_node = MotorsNode("/amazon_robot/cmd_vel", 4, 0.3)
    odometry_node = OdometryNode("/amazon_robot/odom")
    laser_node = LaserNode("/amazon_robot/scan")
    sim_time_node = SimTimeNode()
    platform_listener = PlatformCommandNode("/send_effort")
    platform_pub = PublisherPlatformNode("/send_effort")

    # Spin nodes so that subscription callbacks load topic data
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(odometry_node)
    executor.add_node(laser_node)
    executor.add_node(sim_time_node)
    executor.add_node(platform_listener)
    def __auto_spin() -> None:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0)
            time.sleep(1/freq)
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
    platform_pub.load()

def putdown():
    global liftState
    liftState = False
    platform_pub.unload()

def getLiftState():
    global liftState
    return liftState