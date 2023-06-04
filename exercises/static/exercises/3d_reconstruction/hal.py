import rospy
import cv2
import threading
import time
from datetime import datetime
import math
import numpy as np
from interfaces.camera import ListenerCamera, ListenerParameters


# Hardware Abstraction Layer
class HAL:
    IMG_WIDTH = 640
    IMG_HEIGHT = 480

    def __init__(self):
        rospy.init_node("HAL")

        self.image = None
        self.cameraL = ListenerCamera("/TurtlebotROS/cameraL/image_raw")
        self.cameraR = ListenerCamera("/TurtlebotROS/cameraR/image_raw")

        self.camLeftP = ListenerParameters("3d_reconstruction_conf.yml", "CamACalibration")
        self.camRightP = ListenerParameters("3d_reconstruction_conf.yml", "CamBCalibration")

    # Get Image from ROS Driver Camera
    def getImage(self, lr):
        if (lr == 'left'):
            image = self.cameraL.getImage().data
        elif (lr == 'right'):
            image = self.cameraR.getImage().data
        else:
            print("Invalid camera")

        return image

    # Transform the Coordinate System to the Camera System
    def graficToOptical(self, lr, point2d):
        if (lr == 'left'):
            pointOpt = self.camLeftP.graficToOptical(point2d)
        elif (lr == 'right'):
            pointOpt = self.camRightP.graficToOptical(point2d)
        else:
            print("Invalid camera")
        return pointOpt

    # Backprojects the 2D Point into 3D Space
    def backproject(self, lr, point2d):
        if (lr == 'left'):
            point3D = self.camLeftP.backproject(point2d)
        elif (lr == 'right'):
            point3D = self.camRightP.backproject(point2d)
        else:
            print("Invalid camera")
        return point3D

    # Get Camera Position from ROS Driver Camera
    def getCameraPosition(self, lr):
        if (lr == 'left'):
            position_cam = self.camLeftP.getCameraPosition()
        elif (lr == 'right'):
            position_cam = self.camRightP.getCameraPosition()
        else:
            print("Invalid camera")

        return position_cam

    # Backprojects a 3D Point onto an Image
    def project(self, lr, point3d):
        if (lr == 'left'):
            projected = self.camLeftP.project(point3d)
        elif (lr == 'right'):
            projected = self.camRightP.project(point3d)
        else:
            print("Invalid camera")

        return projected

    # Get Image Coordinates
    def opticalToGrafic(self, lr, point2d):
        if (lr == 'left'):
            point = self.camLeftP.opticalToGrafic(point2d)
        elif (lr == 'right'):
            point = self.camRightP.opticalToGrafic(point2d)
        else:
            print("Invalid camera")

        return point


    def project3DScene(self,point3d):
        px = point3d[0] / 100.0
        py = point3d[1] / 100.0 + 12 
        pz = point3d[2] / 100.0
        outPoint = np.array([px,py,pz]);
        return outPoint

