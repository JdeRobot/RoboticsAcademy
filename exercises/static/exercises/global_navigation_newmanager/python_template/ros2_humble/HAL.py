import rclpy

from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d

# Hardware Abstraction Layer
IMG_WIDTH = 320
IMG_HEIGHT = 240

# ROS2 init
rclpy.create_node('HAL')

pose3d = ListenerPose3d("/taxi_holo/odom")
motors = PublisherMotors("/taxi_holo/cmd_vel", 4, 0.3)

def getPose3d():
    try:
        rclpy.spin_once(pose3d)
        values = pose3d.getPose3d()
        return values
    except Exception as e:
        print(f"Exception in hal getPose3d {repr(e)}")

def setV(velocity):
    motors.sendV(velocity)

def setW(velocity):
    motors.sendW(velocity)