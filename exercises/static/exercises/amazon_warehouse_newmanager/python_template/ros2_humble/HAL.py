import rclpy
import threading
import time

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.specific.amazon_warehouse.platform_controller import PlatformCommandNode, PublisherPlatformNode
from hal_interfaces.specific.amazon_warehouse.simtime import SimTimeNode
# Hardware Abstraction Layer
freq = 30.0

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

    ### HAL INIT ###
    motor_node = MotorsNode("/amazon_robot/cmd_vel", 4, 0.3)
    odometry_node = OdometryNode("/amazon_robot/odom")
    platform_listener = PlatformCommandNode("/send_effort")
    platform_pub = PublisherPlatformNode("/send_effort")
    simtime_node = SimTimeNode("/clock")

    # Spin nodes so that subscription callbacks load topic data
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(odometry_node)
    executor.add_node(platform_listener)
    executor.add_node(simtime_node)
    def __auto_spin() -> None:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0)
            time.sleep(1/freq)
    executor_thread = threading.Thread(target=__auto_spin, daemon=True)
    executor_thread.start()

def getPose3d():
    return odometry_node.getPose3d()

def getSimTime():
    return simtime_node.getSimTime()

def setV(velocity):
    motor_node.sendV(float(velocity))

def setW(velocity):
    motor_node.sendW(float(velocity))

def lift():
    platform_pub.load()

def putdown():
    platform_pub.unload()