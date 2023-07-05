import rclpy
import sys
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
    
    def __init__(self):
        rclpy.init(args=sys.argv)
        rclpy.create_node('HAL')

        self.motors = PublisherMotors("/cmd_vel", 4, 0.3)
        self.pose3d = ListenerPose3d("/odom")
        self.laser = ListenerLaser("/roombaROS/laser/scan")
        self.bumper = ListenerBumper("/roombaROS/events/bumper","roombaROS")

        # Spin nodes so that subscription callbacks load topic data
        # Bumper has to be spinned differently so that GetEntityState works
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.pose3d)
        executor.add_node(self.laser)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        print("HAL-Nodes Thread Started")

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance

    def getBumperData(self):
        self.bumper.spin_bumper_node()
        return self.bumper.getBumperData()

    def getPose3d(self):
        return self.pose3d.getPose3d()

    def getLaserData(self):
        return self.laser.getLaserData()

    def setV(self, velocity):
        self.motors.sendV(velocity)

    def setW(self, velocity):
        self.motors.sendW(velocity)