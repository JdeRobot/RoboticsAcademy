import rclpy
import threading

from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d

# Hardware Abstraction Layer
IMG_WIDTH = 320
IMG_HEIGHT = 240

# ROS2 init
rclpy.create_node('HAL')

pose3d = ListenerPose3d("/odom")
motors = PublisherMotors("/cmd_vel", 4, 0.3)

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(pose3d)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()

print("HAL-Nodes Thread Started")

def getPose3d():
    return pose3d.getPose3d()

def setV(velocity):
    motors.sendV(velocity)

def setW(velocity):
    motors.sendW(velocity)