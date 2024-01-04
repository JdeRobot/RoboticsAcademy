import rospy
import cv2
import threading
import time
from datetime import datetime

from interfaces.camera import ListenerCamera
from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser

# Hardware Abstraction Layer
class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240
    
    
    rospy.init_node("HAL")
    image = None
    camera = ListenerCamera("/F1ROS/cameraL/image_raw")
    motors = PublisherMotors("/F1ROS/cmd_vel", 4, 0.3)
    pose3d = ListenerPose3d("/F1ROS/odom")
    laser = ListenerLaser("/F1ROS/laser/scan")
    	
    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot():
        pass
    @staticmethod
    def setV(velocity):
        HAL.motors.sendV(velocity)
    
    @staticmethod
    def setW(velocity):
        HAL.motors.sendW(velocity)

    @staticmethod
    def getPose3d():
        return HAL.pose3d.getPose3d()

    @staticmethod
    def getLaserData():
        return HAL.laser.getLaserData()

    # Get Image from ROS Driver Camera
    @staticmethod
    def getImage():
        image = HAL.camera.getImage().data
        return image
