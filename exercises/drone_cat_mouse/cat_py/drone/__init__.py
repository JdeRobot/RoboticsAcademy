from camera import *
from cmdvel import *
from extra import *
from pose3d import *
import time


class Drone:
    def __init__(self, topicArming, topicLand, topicSetMode, topicVel, topicPose, topicCamera): 
        self.__camera = ListenerCamera(topicCamera)
        self.__extra = PublisherExtra(topicArming, topicLand, topicSetMode)
        self.__cmdvel = PublisherCMDVel(topicVel)
        self.__pose3d = ListenerPose3d(topicPose)


    def getImage(self):
        return self.__camera.getImage()
    
    def takeoff(self):
        self.__extra.arming()
        self.__cmdvel.sendCMDVel(0,0,2,0,0,0)
        time.sleep(0.5)
        self.__cmdvel.sendCMDVel(0,0,0,0,0,0)
    
    def land(self):
        self.__extra.land()
    
    def toggleCam(self):
        self.__extra.toggleCam()
              
    def reset(self):
        self.__extra.reset()
        
    def record(self, record):
        self.__extra.record(record)

    def sendCMDVel (self, vx,vy,vz,ax,ay,az):
        self.__cmdvel.sendCMDVel(vx,vy,vz,ax,ay,az)
    
    def sendVelocities(self):
        self.__cmdvel.sendVelocities()


    def setVX(self, vx):
        self.__cmdvel.setVX(vx)

    def setVY(self, vy):
        self.__cmdvel.setVY(vy)


    def setVZ(self,vz):
       self.__cmdvel.setVZ(vz)
    
    def setAngularZ(self, az):
        self.__cmdvel.setAngularZ(az)

    def setAngularX(self,ax):
        self.__cmdvel.setAngularX(ax)
        
    def setAngularY(self,ay):
        self.__cmdvel.setAngularY(ay)

    def setYaw(self,yaw):
       self.setAngularZ(yaw)

    def setRoll(self,roll):
        self.setAngularX(roll)
        
    def setPitch(self,pitch):
        self.setAngularY(pitch)

    def getPose3d(self):
        return self.__pose3d.getPose3d()

    def stop(self):
        self.__pose3d.stop()
        self.__camera.stop()

