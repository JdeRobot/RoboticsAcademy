import rclpy
import threading

from interfaces.camera import ListenerCamera
from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser

# Hardware Abstraction Layer
IMG_WIDTH = 320
IMG_HEIGHT = 240

# ROS2 init
rclpy.create_node('HAL')

pose3d = ListenerPose3d("/odom")
motors = PublisherMotors("/cmd_vel", 4, 0.3)
camera = ListenerCamera("/cam_f1_left/image_raw") #TODO: is this necessary?
laser = ListenerLaser("/laser/scan")

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(pose3d)
executor.add_node(laser)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()

print("HAL-Nodes Thread Started")

def getPose3d():
    return pose3d.getPose3d()

def getLaserData():
    return laser.getLaserData()

# Get Image from ROS Driver Camera
def getImage():
    try:
        rclpy.spin_once(camera)
        image = camera.getImage().data
        # image = self._get_test_image()
        # print(f"HAL image set, shape: {image.shape}, bytes: {image.nbytes}", flush=True)
        return image
    except Exception as e:
        print(f"Exception in hal getImage {repr(e)}")

def setV(velocity):
    motors.sendV(velocity)

def setW(velocity):
    motors.sendW(velocity)