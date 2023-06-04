import rospy
import cv2
import threading
import time
from datetime import datetime

from interfaces.camera import ListenerCamera
from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d


# Hardware Abstraction Layer
class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240

    def __init__(self):
        rospy.init_node("HAL")

        self.image = None
        self.camera = ListenerCamera("/TurtlebotROS/cameraR/image_raw")
        self.motors = PublisherMotors("/roombaROS/cmd_vel", 4, 0.3)
        self.pose3d = ListenerPose3d("/roombaROS/odom")

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        pass

    def setV(self, velocity):
        self.motors.sendV(velocity)

    def setW(self, velocity):
        self.motors.sendW(velocity)

    def getPose3d(self):
        return self.pose3d.getPose3d()

    # Get Image from ROS Driver Camera
    def getImage(self):
        image = self.camera.getImage().data
        return image