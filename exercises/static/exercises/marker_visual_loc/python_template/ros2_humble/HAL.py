import rclpy
import sys
import threading
import time

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.noise_odometry import NoisyOdometryNode
from hal_interfaces.general.laser import LaserNode
from hal_interfaces.general.camera import CameraNode

freq = 90.0

def __auto_spin() -> None:
    while rclpy.ok():
        executor.spin_once(timeout_sec=0)
        time.sleep(1/freq)


if not rclpy.ok():
    rclpy.init(args=sys.argv)

### HAL INIT ###
motor_node = MotorsNode("/turtlebot3/cmd_vel", 4, 0.3)
camera_node = CameraNode("/turtlebot3/camera/image_raw")
odometry_node = OdometryNode("/turtlebot3/odom")
laser_node = LaserNode("/turtlebot3/laser/scan")
noisy_odometry_node = NoisyOdometryNode("/turtlebot3/odom")

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(camera_node)
executor.add_node(odometry_node)
executor.add_node(laser_node)
executor.add_node(noisy_odometry_node)

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

# Camera
def getImage():
    image = camera_node.getImage()
    while image == None:
        image = camera_node.getImage()
    return image.data

# Laser
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