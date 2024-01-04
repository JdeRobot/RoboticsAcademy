import rospy
import cv2
import threading
import time
from datetime import datetime

from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser
from interfaces.bumper import ListenerBumper

# Hardware Abstraction Layer
class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240
    
    rospy.init_node("HAL")
    motors = PublisherMotors("/roombaROS/cmd_vel", 4, 0.3)
    pose3d = ListenerPose3d("/roombaROS/odom")
    laser = ListenerLaser("/roombaROS/laser/scan")
    bumper = ListenerBumper("/roombaROS/events/bumper")
    camera_lock = threading.Lock()
    	
    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance
    
    @staticmethod
    def getBumperData():
        return HAL.bumper.getBumperData()
    
    @staticmethod
    def getPose3d():
        return HAL.pose3d.getPose3d()
    
    @staticmethod
    def getLaserData():
        return HAL.laser.getLaserData()
    
    @staticmethod
    def setV(velocity):
        HAL.motors.sendV(velocity)
        
    @staticmethod
    def setW(velocity):
        HAL.motors.sendW(velocity)