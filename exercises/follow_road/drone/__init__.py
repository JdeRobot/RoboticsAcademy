from camera import *
from cmdvel import *
from extra import *
from pose3d import *
import time
import math

import rospy
import mavros
import threading
from mavros import setpoint as SP


class Drone(threading.Thread):
    def __init__(self, topicArming, topicLand, topicTakeOff,topicSetMode, topicVel, topicPose, topicCameraVentral, topicCameraFrontal):
        self.__cameraVentral = ListenerCamera(topicCameraVentral)
        self.__cameraFrontal = ListenerCamera(topicCameraFrontal)
        self.__extra = PublisherExtra(topicArming, topicLand, topicSetMode, topicTakeOff)
        self.__cmdvel = PublisherCMDVel(topicVel)
        self.__pose3d = ListenerPose3d(topicPose)
        rospy.Rate(20)

        mavros.set_namespace()

        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    def getImageVentral(self):
        return self.__cameraVentral.getImage()

    def getImageFrontal(self):
        return self.__cameraFrontal.getImage()

    def takeoff(self):
        self.__extra.get_coordinates()
        self.__extra.arming()
        self.__extra.takeoff()
        self.__extra.change_mode()

    def get_coordinates(self):
        self.__extra.get_coordinates()

    def land(self):
        self.__extra.land()

    def toggleCam(self):
        self.__extra.toggleCam()

    def reset(self):
        self.__extra.reset()

    def record(self, record):
        self.__extra.record(record)

    def sendCMDVel (self,vx,vy,vz,yaw_rate):
        pz = 2 - self.__pose3d.getPose3d().z + vz
        self.__cmdvel.sendCMDVel(0,0,pz,vx,vy,vz,0,0,0,0,yaw_rate)

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
        self.__cameraVentral.stop()
        self.__cameraFrontal.stop()

    def pause (self):
        self.__cmdvel.pause()

    def resume (self):
        self.__cmdvel.resume()
