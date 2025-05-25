import rclpy
import threading
import time

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.noise_odometry import NoisyOdometryNode
from hal_interfaces.general.laser import LaserNode
from hal_interfaces.general.bumper import BumperNode

# Hardware Abstraction Layer
IMG_WIDTH = 320
IMG_HEIGHT = 240

freq = 30.0

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

    ### HAL INIT ###
    motor_node = MotorsNode("/cmd_vel", 4, 0.3)
    odometry_node = OdometryNode("/odom")
    noisy_odometry_node = NoisyOdometryNode("/odom")
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
    executor.add_node(noisy_odometry_node)
    executor.add_node(laser_node)
    def __auto_spin() -> None:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0)
            time.sleep(1/freq)
    executor_thread = threading.Thread(target=__auto_spin, daemon=True)
    executor_thread.start()

# Pose
def getPose3d():
    return odometry_node.getPose3d()

def getOdom():
    return noisy_odometry_node.getPose3d()

# Bumper
def getBumperData():
    try:
        rclpy.spin_once(bumper_node)
        return bumper_node.getBumperData()
    except Exception as e:
        print(f"Exception in hal getBumper {repr(e)}")

def getLaserData():
    laser_data = laser_node.getLaserData()
    while len(laser_data.values) == 0:
        laser_data = laser_node.getLaserData()
    return laser_data

# Linear speed
def setV(v):
    motor_node.sendV(float(v))

# Angular speed
def setW(w):
    motor_node.sendW(float(w))