import rospy
import cv2
import threading
import time
from datetime import datetime

from interfaces.camera import ListenerCamera
from interfaces.motors import PublisherMotors
from shared.image import SharedImage

# Hardware Abstraction Layer
class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240
    
    def __init__(self):
    	rospy.init_node("HAL")
    
    	self.shared_image = SharedImage("halimage")
    	self.camera = ListenerCamera("/F1ROS/cameraL/image_raw")
    	self.motors = PublisherMotors("/F1ROS/cmd_vel", 4, 0.3)
    	
    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance
    
    # Get Image from ROS Driver Camera
    def getImage(self):
        image = self.camera.getImage().data
        self.shared_image.add(image)
			
