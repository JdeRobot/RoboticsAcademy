
import sys, traceback, Ice
import easyiceconfig as EasyIce
import jderobot
import numpy as np
import threading
import math

class Sensor:
    def __init__(self):
        self.lock = threading.Lock()
        self.playButton=False

        try:
            ic = EasyIce.initialize(sys.argv)
            properties = ic.getProperties()
            basecameraL = ic.propertyToProxy("ObstacleAvoidance.CameraLeft.Proxy")
            self.cameraProxyL = jderobot.CameraPrx.checkedCast(basecameraL)

            if self.cameraProxyL:
                self.imageLeft = self.cameraProxyL.getImageData("RGB8")
                self.imageLeft_h= self.imageLeft.description.height
                self.imageLeft_w = self.imageLeft.description.width
            else:
                print 'Interface for left camera not connected'

            basecameraR = ic.propertyToProxy("ObstacleAvoidance.CameraRight.Proxy")
            self.cameraProxyR = jderobot.CameraPrx.checkedCast(basecameraR)

            if self.cameraProxyR:
                self.imageRight = self.cameraProxyR.getImageData("RGB8")
                self.imageRight_h= self.imageRight.description.height
                self.imageRight_w = self.imageRight.description.width
            else:
                print 'Interface for right camera not connected'

            motorsBase = ic.propertyToProxy("ObstacleAvoidance.motors.Proxy")
            self.motorsProxy = jderobot.MotorsPrx.checkedCast(motorsBase)
            if self.motorsProxy:
                print 'Interface for motors connected'
            else:
                print 'Interface for motors not connected'

            encodersBase = ic.propertyToProxy("ObstacleAvoidance.encoders.Proxy")
            #self.encodersProxy = jderobot.EncodersPrx.checkedCast(encodersBase)
            self.encodersProxy = jderobot.Pose3DPrx.checkedCast(encodersBase)
            if self.encodersProxy:
                print 'Interface for encoders connected'
            else:
                print 'Interface for encoders not connected'

            laserBase = ic.propertyToProxy("ObstacleAvoidance.laser.Proxy")
            self.laserProxy = jderobot.LaserPrx.checkedCast(laserBase)
            if self.laserProxy:
                print 'Interface for laser connected'
            else:
                print 'Interface for laser not connected'

            self.maxSpeedV=2.0
            self.maxSpeedW=-1.0

        except:
            traceback.print_exc()
            exit()
            status = 1

            
    def update(self):
        self.lock.acquire()
        self.updateCameras()
        self.lock.release()

    def updateCameras(self):
        if self.cameraProxyL:
            self.imageLeft = self.cameraProxyL.getImageData("RGB8")
            self.imageLeft_h= self.imageLeft.description.height
            self.imageLeft_w = self.imageLeft.description.width

        if self.cameraProxyR:
            self.imageRight = self.cameraProxyR.getImageData("RGB8")
            self.imageRight_h= self.imageRight.description.height
            self.imageRight_w = self.imageRight.description.width
    
    def getImageLeft(self):
        if self.cameraProxyL:
            self.lock.acquire()
            imgL = np.zeros((self.imageLeft_h, self.imageLeft_w, 3), np.uint8)
            imgL = np.frombuffer(self.imageLeft.pixelData, dtype=np.uint8)
            imgL.shape = self.imageLeft_h, self.imageLeft_w, 3
            self.lock.release()
            return imgL;

        return None

    def getImageRight(self):
        if self.cameraProxyR:
            self.lock.acquire()
            imgR = np.zeros((self.imageRight_h, self.imageRight_w, 3), np.uint8)
            imgR = np.frombuffer(self.imageRight.pixelData, dtype=np.uint8)
            imgR.shape = self.imageRight_h, self.imageRight_w, 3
            self.lock.release()
            return imgR;

        return None

    def isPlayButton(self):
        return self.playButton
    
    def setPlayButton(self,value):
        if (value==False):
            self.setV(0)
            self.setW(0)
        self.playButton=value

    def isVirtual(self):
        return self.virtualDrone

    def setV(self,v):
        normalizedV=v*self.maxSpeedV
        self.motorsProxy.setV(normalizedV)

    def setW(self,w):
        normalizedW=w*self.maxSpeedW
        self.motorsProxy.setW(normalizedW)

    def getV(self):
        return self.motorsProxy.getV()

    def getW(self):
        return self.motorsProxy.getW()

    def stop(self):
        self.motorsProxy.setV(0)
        self.motorsProxy.setW(0)

    def getRobotX(self):
        data = self.encodersProxy.getPose3DData()
        return data.x/1000.0

    def getRobotY(self):
        data = self.encodersProxy.getPose3DData()
        return data.y/1000.0

    def getRobotTheta(self):
        d = self.encodersProxy.getPose3DData()
        theta = math.atan2(2*(d.q0*d.q3 + d.q1*d.q2), 1-2*(d.q2*d.q2 + d.q3*d.q3))
        return theta

    def getLaserData(self):
        return self.laserProxy.getLaserData()


