import rclpy
import sys
import cv2
import threading
import time
from datetime import datetime

from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser

# Hardware Abstraction Layer
class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240

    rclpy.init(args=sys.argv)
    rclpy.create_node('HAL')

    motors = PublisherMotors("/prius_autoparking/cmd_vel", 4, 0.3)
    laser_front = ListenerLaser("/prius_autoparking/scan_front")
    laser_right = ListenerLaser("/prius_autoparking/scan_side")
    laser_back = ListenerLaser("/prius_autoparking/scan_back")
    pose3d = ListenerPose3d("/prius_autoparking/odom")

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @staticmethod
    def getPose3d():
        return HAL.pose3d.getPose3d()

    @staticmethod
    def getFrontLaserData():
        return HAL.laser_front.getLaserData()

    @staticmethod
    def getRightLaserData():
        return HAL.laser_right.getLaserData()

    @staticmethod
    def getBackLaserData():
        return HAL.laser_back.getLaserData()

    @staticmethod
    def setV(velocity):
        HAL.motors.sendV(velocity)

    @staticmethod
    def setW(velocity):
        HAL.motors.sendW(velocity)