import rclpy
import threading
import time
from hal_interfaces.general.camera import CameraNode
from hal_interfaces.specific.threed_reconstruction.parameters_camera import ListenerParameters
import numpy as np


# Hardware Abstraction Layer

IMG_WIDTH = 640
IMG_HEIGHT = 480
image = None
freq = 15.0


# ROS2 init
if not rclpy.ok():
    rclpy.init(args=None)

cameraL = CameraNode("/cam_turtlebot_left/image_raw")
cameraR = CameraNode("/cam_turtlebot_right/image_raw")

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(cameraL)
executor.add_node(cameraR)
def __auto_spin() -> None:
    while rclpy.ok():
        executor.spin_once(timeout_sec=0)
        time.sleep(1/freq)
executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()

camLeftP = ListenerParameters("3d_reconstruction_conf.yml", "CamACalibration")
camRightP = ListenerParameters("3d_reconstruction_conf.yml", "CamBCalibration")

# Get Image from ROS Driver Camera
def getImage(lr):
    try:
        if (lr == 'left'):
            image = cameraL.getImage()
            while image == None:
                image = cameraL.getImage()
        elif (lr == 'right'):
            image = cameraR.getImage()
            while image == None:
                image = cameraR.getImage()
        return image.data
    except Exception as e:
        print(f"Exception in hal getImage {repr(e)}")

# Transform the Coordinate System to the Camera System
def graficToOptical(lr, point2d):
    if (lr == 'left'):
        pointOpt = camLeftP.graficToOptical(point2d)
    elif (lr == 'right'):
        pointOpt = camRightP.graficToOptical(point2d)
    else:
        print("Invalid camera")
    return pointOpt

# Backprojects the 2D Point into 3D Space
def backproject(lr, point2d):
    if (lr == 'left'):
        point3D = camLeftP.backproject(point2d)
    elif (lr == 'right'):
        point3D = camRightP.backproject(point2d)
    else:
        print("Invalid camera")
    return point3D

# Get Camera Position from ROS Driver Camera
def getCameraPosition(lr):
    if (lr == 'left'):
        position_cam = camLeftP.getCameraPosition()
    elif (lr == 'right'):
        position_cam = camRightP.getCameraPosition()
    else:
        print("Invalid camera")

    return position_cam

# Backprojects a 3D Point onto an Image
def project(lr, point3d):
    if (lr == 'left'):
        projected = camLeftP.project(point3d)
    elif (lr == 'right'):
        projected = camRightP.project(point3d)
    else:
        print("Invalid camera")

    return projected

# Get Image Coordinates
def opticalToGrafic(lr, point2d):
    if (lr == 'left'):
        point = camLeftP.opticalToGrafic(point2d)
    elif (lr == 'right'):
        point = camRightP.opticalToGrafic(point2d)
    else:
        print("Invalid camera")

    return point

def project3DScene(point3d):
    px = point3d[0] / 100.0
    py = point3d[1] / 100.0 + 12 
    pz = point3d[2] / 100.0
    outPoint = np.array([px,py,pz]);
    return outPoint
