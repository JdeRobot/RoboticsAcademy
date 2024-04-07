import rclpy
import threading

from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser
from interfaces.platform_controller import PlatformCommandListener
from interfaces.platform_publisher import PublisherPlatform

# Hardware Abstraction Layer
# rclpy.init(args=sys.argv)
rclpy.create_node('HAL')

motors = PublisherMotors("/amazon_robot/cmd_vel", 4, 0.3)
pose3d = ListenerPose3d("/amazon_robot/odom")
laser = ListenerLaser("/amazon_robot/scan")
platform_listener = PlatformCommandListener()
platform_pub = PublisherPlatform("/send_effort")

# Spin nodes so that subscription callbacks load topic data
# Bumper has to be spinned differently so that GetEntityState works
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(pose3d)
executor.add_node(laser)
executor.add_node(platform_listener)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()

print("HAL-Nodes Thread Started")

def getPose3d():
    return pose3d.getPose3d()

def getLaserData():
    return laser.getLaserData()

def setV(velocity):
    motors.sendV(velocity)

def setW(velocity):
    motors.sendW(velocity)

def load():
    platform_pub.load()

def unload():
    platform_pub.unload()