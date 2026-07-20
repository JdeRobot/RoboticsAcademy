import rclpy
import sys
import threading
import time

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode
from hal_interfaces.general.camera import CameraNode
from hal_interfaces.general.classnet import NeuralNetwork

### HAL INIT ###

freq = 30.0


# Mutes exceptions
def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=sys.argv)

motor_node = MotorsNode("/turtlebot2/cmd_vel", 4, 0.3)
odometry_node = OdometryNode("/turtlebot2/odom")
laser_node = LaserNode("/turtlebot2/laser/scan")
camera_node = CameraNode("/turtlebot2/camera/image_raw")
neural_network = NeuralNetwork()

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(odometry_node)
executor.add_node(laser_node)
executor.add_node(camera_node)


def __auto_spin() -> None:
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0)
        except Exception:
            pass
        time.sleep(1 / freq)


executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()


### GETTERS ###


# Laser
def getLaserData():
    laser = laser_node.getLaserData()
    timestamp = laser.timeStamp
    while timestamp == 0.0:
        laser = laser_node.getLaserData()
        timestamp = laser.timeStamp
    return laser


# Pose
def getPose3d():
    return odometry_node.getPose3d()


# Image
def getImage():
    image = camera_node.getImage()
    while image is None:
        image = camera_node.getImage()
    return image.data


# Bounding boxes
def getBoundingBoxes(img):
    return neural_network.getBoundingBoxes(img)


### SETTERS ###


# Linear speed
def setV(v):
    motor_node.sendV(v)


# Angular speed
def setW(w):
    motor_node.sendW(w)
