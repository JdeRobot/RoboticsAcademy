import rospy
import cv2
import threading
import time
import os
import yaml
from datetime import datetime

from interfaces.camera import ListenerCamera
from interfaces.pose3d import ListenerPose3d
from interfaces.motors import PublisherMotors

# Hardware Abstraction Layer
class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240

    def __init__(self):
        rospy.init_node("HAL")

        # Shared memory variables
        self.image = None
        self.v = None
        self.w = None
        self.p3d = None

        if os.getcwd() == "/":
            f = open("/RoboticsAcademy/exercises/car_junction/web-template/stop_conf.yml", "r")
        else:
            f = open("stop_conf.yml", "r")

        cfg = yaml.load(f)

        ymlNode = cfg['Stop']
        #node = rospy.init_node(ymlNode["NodeName"], anonymous=True)

        # ------------ M O T O R S ----------------------------------
        print("Publishing "+  "Stop.Motors" + " with ROS messages")
        topicM = cfg['Stop']["Motors"]["Topic"]
        maxW = cfg['Stop']["Motors"]["maxW"]
        if not maxW:
            maxW = 0.5
            print ("Stop.Motors"+".maxW not provided, the default value is used: "+ repr(maxW))

        maxV = cfg['Stop']["Motors"]["maxV"]
        if not maxV:
            maxV = 5
            print ("Stop.Motors"+".maxV not provided, the default value is used: "+ repr(maxV))
    
        self.motors = PublisherMotors(topicM, maxV, maxW)

        # ----------------- P O S E     3 D -------------------------------------
        print("Receiving " + "Stop.Pose3D" + " from ROS messages")
        topicP = cfg['Stop']["Pose3D"]["Topic"]
        self.pose3d = ListenerPose3d(topicP)

        # -------- C A M E R A C E N T R A L --------------------------------------
        print("Receiving " + "Stop.CameraC" + "  CameraData from ROS messages")
        topicCameraC  = cfg['Stop']["CameraC"]["Topic"]
        self.cameraC = ListenerCamera(topicCameraC)

        # -------- C A M E R A L E F T --------------------------------------------
        print("Receiving " + "Stop.CameraL" + "  CameraData from ROS messages")
        topicCameraL  = cfg['Stop']["CameraL"]["Topic"]
        self.cameraL = ListenerCamera(topicCameraL)

        # -------- C A M E R A R I G H T ------------------------------------------
        print("Receiving " + "Stop.CameraR" + "  CameraData from ROS messages")
        topicCameraR  = cfg['Stop']["CameraR"]["Topic"]
        self.cameraR = ListenerCamera(topicCameraR)

        # print("Starting movement of dummy cars")
        #motorsDummy1 = PublisherMotors("/dummy1/cmd_vel", 4, 0.3)
        #motorsDummy2 = PublisherMotors("/dummy2/cmd_vel", 4, 0.3)

        #motorsDummy1.sendV(2.5)
        #motorsDummy2.sendV(3)


    # Get Image from ROS Driver Camera
    def getImage(self, lr):
        if (lr == 'left'):
            image = self.cameraL.getImage().data
        elif (lr == 'right'):
            image = self.cameraR.getImage().data
        elif (lr == 'center'):
            image = self.cameraC.getImage().data
        else:
            print("Invalid camera")

        return image

    # Set the velocity
    def setV(self, velocity):
        self.v = velocity
        self.motors.sendV(velocity)

    # Get the velocity
    def getV(self):
        velocity = self.v
        return velocity

    # Get the angular velocity
    def getW(self):
        angular = self.w
        return angular

    # Set the angular velocity
    def setW(self, angular):
        self.w = angular
        self.motors.sendW(angular)

    def getPose3D(self):
        return self.pose3d

    def setPose3D(self, pose3d):
        self.pose3d = pose3d
