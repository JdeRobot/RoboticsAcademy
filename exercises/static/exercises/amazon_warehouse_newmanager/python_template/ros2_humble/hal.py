import rclpy
import sys
import cv2
import threading

from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser
from interfaces.platform_controller import PlatformCommandListener
from interfaces.platform_publisher import PublisherPlatform


# Hardware Abstraction Layer
class HAL:
    
    def __init__(self):
        rclpy.init(args=sys.argv)
        rclpy.create_node('HAL')

        self.motors = PublisherMotors("/amazon_robot/cmd_vel", 4, 0.3)
        self.pose3d = ListenerPose3d("/amazon_robot/odom")
        self.laser = ListenerLaser("/amazon_robot/scan")
        self.platform_listener = PlatformCommandListener()
        self.platform_pub = PublisherPlatform("/send_effort")

        # Spin nodes so that subscription callbacks load topic data
        # Bumper has to be spinned differently so that GetEntityState works
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.pose3d)
        executor.add_node(self.laser)
        executor.add_node(self.platform_listener)
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

    def load(self):
        self.platform_pub.load()

    def unload(self):
        self.platform_pub.unload()
