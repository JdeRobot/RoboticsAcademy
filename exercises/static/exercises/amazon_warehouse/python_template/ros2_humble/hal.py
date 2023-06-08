import rclpy
import sys
import cv2
import threading
import time
from datetime import datetime

from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser
from interfaces.camera import ListenerCamera

# Hardware Abstraction Layer
class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240
    
    def __init__(self):
        rclpy.init(args=sys.argv)
        rclpy.create_node('HAL')

        self.motors = PublisherMotors("/amazon_robot/cmd_vel", 4, 0.3)
        self.pose3d = ListenerPose3d("/amazon_robot/odom")
        self.laser = ListenerLaser("/amazon_robot/scan")
        self.camera = ListenerCamera("/amazon_robot/camera_front/image_raw")

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

    def getPose3d(self):
        return self.pose3d.getPose3d()

    def getLaserData(self):
        return self.laser.getLaserData()

    def setV(self, velocity):
        self.motors.sendV(velocity)

    def setW(self, velocity):
        self.motors.sendW(velocity)

    def getImage(self):
        try:
            rclpy.spin_once(self.camera)
            image = self.camera.getImage().data
            return image
        except Exception as e:
            print(f"Exception in hal getImage {repr(e)}")