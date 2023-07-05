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
    
    def __init__(self):
        rclpy.init(args=sys.argv)
        rclpy.create_node('HAL')

        self.motors = PublisherMotors("/prius_autoparking/cmd_vel", 4, 0.3)
        self.laser_front = ListenerLaser("/prius_autoparking/scan_front")
        self.laser_right = ListenerLaser("/prius_autoparking/scan_side")
        self.laser_back = ListenerLaser("/prius_autoparking/scan_back")
        self.pose3d = ListenerPose3d("/prius_autoparking/odom")

        # Spin nodes so that subscription callbacks load topic data
        # Bumper has to be spinned differently so that GetEntityState works
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.pose3d)
        executor.add_node(self.laser_front)
        executor.add_node(self.laser_right)
        executor.add_node(self.laser_back)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        print("HAL-Nodes Thread Started")

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance
    
    def getPose3d(self):
        return self.pose3d.getPose3d()

    def getFrontLaserData(self):
        return self.laser_front.getLaserData()

    def getRightLaserData(self):
        return self.laser_right.getLaserData()

    def getBackLaserData(self):
        return self.laser_back.getLaserData()

    def setV(self, velocity):
        self.motors.sendV(velocity)

    def setW(self, velocity):
        self.motors.sendW(velocity)