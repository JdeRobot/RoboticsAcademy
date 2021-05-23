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
    
    def __init__(self):
    	rospy.init_node("HAL")
    
    	self.image = None
    	self.camera = ListenerCamera("/F1ROS/cameraL/image_raw")
    	self.motors = PublisherMotors("/F1ROS/cmd_vel", 4, 0.3)
    	self.pose3d = ListenerPose3d("/F1ROS/odom")
    	self.laser = ListenerLaser("/F1ROS/laser/scan")
    	
    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(self):
        pass
    
    # Get Image from ROS Driver Camera
    def getImage(self):
        image = self.camera.getImage().data
        return image
