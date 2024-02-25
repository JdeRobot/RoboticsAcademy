import rclpy
import sys

from interfaces.camera import ListenerCamera
from interfaces.laser import ListenerLaser
from user_functions import HALFunctions

IMG_WIDTH = 320
IMG_HEIGHT = 240

print("HAL initializing", flush=True)
rclpy.init(args=sys.argv)

motor = MotorDriver("/cmd_vel", 4, 0.3)
camera = ListenerCamera("/depth_camera/image_raw")
laser = ListenerLaser("/scan")
odometry = ListenerPose3d("/odom")

# Get laser data from ROS Driver
def getLaserData():
    try:
        rclpy.spin_once(laser)
        values = laser.getLaserData().values
        return values
    except Exception as e:
        print(f"Exception in hal getLaserData {repr(e)}")

# Get pose from ROS Driver 
def getPose3d():
    try:
        rclpy.spin_once(odometry)
        pose = odometry.getPose3d()
        return pose
    except Exception as e:
        print(f"Exception in hal getPose3d {repr(e)}")

# Get Image from ROS Driver Camera
def getImage():
    try:
        rclpy.spin_once(camera)
        image = camera.getImage().data
        return image
    except Exception as e:
        print(f"Exception in hal getImage {repr(e)}")

# Set the linear velocity
def setV(v):
    motor.sendV(v)

# Set the angular velocity
def setW(w):
    motor.sendW(w)

# Get the velocity
def getV(self):
    return last_twist.linear.x