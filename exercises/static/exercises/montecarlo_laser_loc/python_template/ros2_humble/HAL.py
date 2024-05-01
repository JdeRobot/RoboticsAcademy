import rclpy
import threading

from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser
from interfaces.bumper import ListenerBumper


# Hardware Abstraction Layer
IMG_WIDTH = 320
IMG_HEIGHT = 240

rclpy.create_node('HAL')

motors = PublisherMotors("/cmd_vel", 4, 0.3)
pose3d = ListenerPose3d("/odom")
laser = ListenerLaser("/roombaROS/laser/scan")
bumper = ListenerBumper("/roombaROS/events/bumper","roombaROS")

# Spin nodes so that subscription callbacks load topic data
# Bumper has to be spinned differently so that GetEntityState works
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(pose3d)
executor.add_node(laser)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()

print("HAL-Nodes Thread Started")

def getBumperData():
    bumper.spin_bumper_node()
    return bumper.getBumperData()

def getPose3d():
    return pose3d.getPose3d()

def getLaserData():
    laser_data = laser.getLaserData()
    while len(laser_data.values) == 0:
        laser_data = laser.getLaserData()
    return laser_data

def setV(velocity):
    motors.sendV(velocity)

def setW(velocity):
    motors.sendW(velocity)