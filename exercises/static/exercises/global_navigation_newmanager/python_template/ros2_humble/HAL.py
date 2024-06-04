import rclpy
import threading

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode

# Hardware Abstraction Layer
IMG_WIDTH = 320
IMG_HEIGHT = 240

# ROS2 init
if not rclpy.ok():
    rclpy.init(args=None)
    rclpy.create_node('HAL')

pose3d = OdometryNode("/odom")
motors = MotorsNode("/cmd_vel", 4, 0.3)

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(pose3d)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()

# Pose
def getPose3d():
    return pose3d.getPose3d()

def setV(velocity):
    motors.sendV(float(velocity))

def setW(velocity):
    motors.sendW(float(velocity))