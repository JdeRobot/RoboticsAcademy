import rclpy
import sys

from hal_interfaces.motors import MotorsNode
from hal_interfaces.odometry import OdometryNode
from hal_interfaces.laser import LaserNode
from hal_interfaces.camera import CameraNode
import hal_interfaces.neural_network

### HAL INIT ###

print("HAL initializing", flush=True)
rclpy.init(args=sys.argv)

motor_node = MotorsNode("/cmd_vel", 4, 0.3)
odometry_node = OdometryNode("/odom")
laser_node = LaserNode("/scan")
camera_node = CameraNode("/depth_camera/image_raw")
neural_network = hal_interfaces.neural_network.NeuralNetwork()

### GETTERS ###

# Laser
def getLaserData():
    try:
        rclpy.spin_once(laser_node)
        values = laser_node.getLaserData().values
        return values
    except Exception as e:
        print(f"Exception in hal getLaserData {repr(e)}")

# Pose
def getPose3d():
    try:
        rclpy.spin_once(odometry_node)
        pose = odometry_node.getPose3d()
        return pose
    except Exception as e:
        print(f"Exception in hal getPose3d {repr(e)}")

# Image
def getImage():
    try:
        rclpy.spin_once(camera_node)
        image = camera_node.getImage().data
        return image
    except Exception as e:
        print(f"Exception in hal getImage {repr(e)}")

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