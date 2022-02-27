import rospy
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
    
    def __init__(self):
        rospy.init_node("HAL")
    
        self.motors = PublisherMotors("/taxi_holo/cmd_vel", 4, 0.3)
        self.pose3d = ListenerPose3d("/taxi_holo/odom")
        self.laser_front = ListenerLaser("/F1ROS/laser_f/scan")
        self.laser_right = ListenerLaser("/F1ROS/laser_r/scan")
        self.laser_back = ListenerLaser("/F1ROS/laser_b/scan")
    
    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(self):
        pass
    
    def setV(self, velocity):
        self.motors.sendV(velocity)
    
    def setW(self, velocity):
        self.motors.sendW(velocity)

    def getPose3d(self):
        return self.pose3d.getPose3d()

    def getFrontLaserData(self):
        return self.laser_front.getLaserData()

    def getRightLaserData(self):
        return self.laser_right.getLaserData()

    def getBackLaserData(self):
        return self.laser_back.getLaserData()
