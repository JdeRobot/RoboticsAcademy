import rclpy
from interfaces.camera import ListenerCamera, ListenerParameters
import numpy as np


# Hardware Abstraction Layer

IMG_WIDTH = 640
IMG_HEIGHT = 480

# ROS2 init
rclpy.create_node('HAL')

image = None
cameraL = ListenerCamera("/cam_turtlebot_left/image_raw")
cameraR = ListenerCamera("/cam_turtlebot_right/image_raw")

camLeftP = ListenerParameters("3d_reconstruction_conf.yml", "CamACalibration")
camRightP = ListenerParameters("3d_reconstruction_conf.yml", "CamBCalibration")

# Get Image from ROS Driver Camera
def getImage(lr):
    if (lr == 'left'):
        image = cameraL.getImage().data
    elif (lr == 'right'):
        image = cameraR.getImage().data
    else:
        print("Invalid camera")

    return image

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

