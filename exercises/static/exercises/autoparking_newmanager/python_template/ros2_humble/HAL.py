import rclpy

from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser

# Hardware Abstraction Layer
IMG_WIDTH = 320
IMG_HEIGHT = 240

# ROS2 init
rclpy.create_node('HAL')

motors = PublisherMotors("/prius_autoparking/cmd_vel", 4, 0.3)
laser_front = ListenerLaser("/prius_autoparking/scan_front")
laser_right = ListenerLaser("/prius_autoparking/scan_side")
laser_back = ListenerLaser("/prius_autoparking/scan_back")
pose3d = ListenerPose3d("/prius_autoparking/odom")


def getPose3d():
    try:
        rclpy.spin_once(pose3d)
        values = pose3d.getPose3d()
        return values
    except Exception as e:
        print(f"Exception in hal getPose3d {repr(e)}")

def getFrontLaserData():
    try:
        rclpy.spin_once(laser_front)
        values = laser_front.getLaserData().values
        return values
    except Exception as e:
        print(f"Exception in hal getFrontLaserData {repr(e)}")

def getRightLaserData():
    try:
        rclpy.spin_once(laser_right)
        values = laser_right.getLaserData().values
        return values
    except Exception as e:
        print(f"Exception in hal getRightLaserData {repr(e)}")

def getBackLaserData():
    try:
        rclpy.spin_once(laser_back)
        values = laser_back.getLaserData().values
        return values
    except Exception as e:
        print(f"Exception in hal getBackLaserData {repr(e)}")

def setV(velocity):
    motors.sendV(velocity)

def setW(velocity):
    motors.sendW(velocity)