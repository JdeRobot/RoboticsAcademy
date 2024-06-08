import rclpy
import sys
import threading
import time

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode
from hal_interfaces.general.bumper import BumperNode


freq = 30.0

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=sys.argv)

    ### HAL INIT ###
    motor_node = MotorsNode("/cmd_vel", 4, 0.3)
    odometry_node = OdometryNode("/odom")
    laser_node = LaserNode("/roombaROS/laser/scan")
    bumper_node = BumperNode(
        [
            "/roombaROS/events/right_bumper",
            "/roombaROS/events/center_bumper",
            "/roombaROS/events/left_bumper",
        ]
    )

    # Spin nodes so that subscription callbacks load topic data
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(odometry_node)
    executor.add_node(laser_node)
    executor.add_node(bumper_node) 
    def __auto_spin() -> None:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0)
            time.sleep(1/freq)
    executor_thread = threading.Thread(target=__auto_spin, daemon=True)
    executor_thread.start()


### GETTERS ###

# Laser
def getLaserData():
    try:
        return laser_node.getLaserData()
    except Exception as e:
        print(f"Exception in hal getLaserData {repr(e)}")

# Pose
def getPose3d():
    try:
        return odometry_node.getPose3d()
    except Exception as e:
        print(f"Exception in hal getPose3d {repr(e)}")        

# Bumper
def getBumperData():
    try:
        return bumper_node.getBumperData()
    except Exception as e:
        print(f"Exception in hal getBumper {repr(e)}")


### SETTERS ###

# Linear speed
def setV(v):
    motor_node.sendV(float(v))

# Angular speed
def setW(w):
    motor_node.sendW(float(w))