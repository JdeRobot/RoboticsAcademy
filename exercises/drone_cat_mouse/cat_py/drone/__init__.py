from camera import *
from cmdvel import *
from extra import *
from pose3d import *
import time
import math


class Drone:
    def __init__(self, topicArming, topicLand, topicSetMode, topicVel, topicPose, topicCameraVentral, topicCameraFrontal): 
        self.__cameraVentral = ListenerCamera(topicCameraVentral)
        self.__cameraFrontal = ListenerCamera(topicCameraFrontal)
        self.__extra = PublisherExtra(topicArming, topicLand, topicSetMode)
        self.__cmdvel = PublisherCMDVel(topicVel)
        self.__pose3d = ListenerPose3d(topicPose)


    def getImageVentral(self):
        return self.__cameraVentral.getImage()

    def getImageFrontal(self):
        return self.__cameraFrontal.getImage()
    
    def takeoff(self):
        self.__extra.arming()
        self.__cmdvel.sendCMDVel(0,0,2,0,0,0)
        time.sleep(0.5)
        self.__cmdvel.sendCMDVel(0,0,0,0,0,0)
        time.sleep(0.1)
        self.sendCMDVel(1,0,0,0,0,0)
    
    def land(self):
        self.__extra.land()
    
    def toggleCam(self):
        self.__extra.toggleCam()
              
    def reset(self):
        self.__extra.reset()
        
    def record(self, record):
        self.__extra.record(record)

    def sendCMDVel (self, vx,vy,vz,ax,ay,az):
        pose = self.__pose3d.getPose3d()
        yaw = pose.yaw
        vxt = vx*math.cos(yaw) - vy*math.sin(yaw)
        vyt = vx*math.sin(yaw) + vy * math.cos(yaw)
        self.__cmdvel.sendCMDVel(vxt,vyt,vz,ax,ay,az)

    def getPose3d(self):
        return self.__pose3d.getPose3d()

    def stop(self):
        self.__pose3d.stop()
        self.__cameraVentral.stop()
        self.__cameraFrontal.stop()

