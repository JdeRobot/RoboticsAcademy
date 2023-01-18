import rospy
import cv2
import threading
import time
from datetime import datetime

from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d


# Hardware Abstraction Layer
class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240
    
    def __init__(self):
        rospy.init_node("HAL")

        self.pose3d = ListenerPose3d("/taxi_holo/odom")
        self.motors = PublisherMotors("/taxi_holo/cmd_vel", 4, 0.3)
        self.camera_lock = threading.Lock()
  
    def setV(self, velocity):
        self.motors.sendV(velocity)

    def setW(self, velocity):
        self.motors.sendW(velocity)

    def getPose3d(self):
        return self.pose3d.getPose3d()
