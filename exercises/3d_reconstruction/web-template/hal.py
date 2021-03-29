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
    IMG_WIDTH = 320
    IMG_HEIGHT = 240

    def __init__(self):
        rospy.init_node("HAL")

        self.image = None
        self.cameraL = ListenerCamera("/TurtlebotROS/cameraL/image_raw")
        self.cameraR = ListenerCamera("/TurtlebotROS/cameraR/image_raw")

        self.camLeftP = ListenerParameters("3d_reconstruction_conf.yml", "CamACalibration")
        self.camRightP = ListenerParameters("3d_reconstruction_conf.yml", "CamBCalibration")

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance

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
    def opticalToGrafic(self,lr, point2d):
        if (lr == 'left'):
            point = self.camLeftP.opticalToGrafic(point2d)
        elif (lr == 'right'):
            point = self.camRightP.opticalToGrafic(point2d)
        else:
            print("Invalid camera")

        return point


    def project3DScene(self,point3d):
        phi = 90
        tetha = 0
        alpha = -90
        cos_phi = math.cos(math.radians(phi))
        sin_phi = math.sin(math.radians(phi))
        cos_tetha = math.cos(math.radians(tetha))
        sin_tetha = math.sin(math.radians(tetha))
        cos_alpha = math.cos(math.radians(alpha))
        sin_alpha = math.sin(math.radians(alpha))
        px = ((cos_phi*cos_tetha)*point3d[0] + (sin_phi*sin_alpha - cos_phi*sin_tetha*cos_alpha)*point3d[1] + (cos_phi*sin_tetha*sin_alpha + sin_phi*cos_alpha)*point3d[2])/100.0
        py = (sin_tetha*point3d[0] + (cos_tetha*cos_alpha)*point3d[1] + (-cos_tetha*sin_alpha)*point3d[2])/100.0 + 12 
        pz = ((-sin_phi*cos_tetha)*point3d[0] + (sin_phi*sin_tetha*cos_alpha + cos_phi*sin_alpha)*point3d[1] + (cos_phi*cos_alpha - sin_phi*sin_tetha*sin_alpha)*point3d[2])/100.0
        
        outPoint = np.array([px,py,pz]);
        return outPoint

