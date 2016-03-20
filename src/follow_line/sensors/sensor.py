
import sys, traceback, Ice
import easyiceconfig as EasyIce
import jderobot
import numpy as np
import threading

class Sensor:
    def __init__(self):
        self.lock = threading.Lock()
        self.playButton=False

        try:
            ic = EasyIce.initialize(sys.argv)
            properties = ic.getProperties()
            basecameraL = ic.propertyToProxy("FollowLine.CameraLeft.Proxy")
            self.cameraProxyL = jderobot.CameraPrx.checkedCast(basecameraL)

            self.robot = properties.getProperty("FolowLine.robot")

            if self.cameraProxyL:
                self.imageLeft = self.cameraProxyL.getImageData("RGB8")
                self.imageLeft_h= self.imageLeft.description.height
                self.imageLeft_w = self.imageLeft.description.width
            else:
                print 'Interface for left camera not connected'

            basecameraR = ic.propertyToProxy("FollowLine.CameraRight.Proxy")
            self.cameraProxyR = jderobot.CameraPrx.checkedCast(basecameraR)

            if self.cameraProxyR:
                self.imageRight = self.cameraProxyR.getImageData("RGB8")
                self.imageRight_h= self.imageRight.description.height
                self.imageRight_w = self.imageRight.description.width
            else:
                print 'Interface for right camera not connected'


            motorsBase = ic.propertyToProxy("FolowLine.motors.Proxy")
            self.motorsProxy = jderobot.MotorsPrx.checkedCast(motorsBase)
            if self.motorsProxy:
                print 'Interface for motors connected'
            else:
                print 'Interface for motors not connected'


            if self.robot == 'F1':
                self.maxSpeedV=30
                self.maxSpeedW=3
            else:
                self.maxSpeedV=100
                self.maxSpeedW=100


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

    def setV(self,v,percentage=False):
        myV=v

        if (percentage or self.robot=='Pioneer'):
            myV=myV*self.maxSpeedV

        self.motorsProxy.setV(myV)

    def setW(self,w,percentage=False):
        myW=w
        if self.robot =='F1':
            myW=-myW

        if (percentage or self.robot=='Pioneer'):
            myW=myW*self.maxSpeedW

        self.motorsProxy.setW(myW)


    def getV(self):
        return self.motorsProxy.getV()

    def getW(self):
        return self.motorsProxy.getW()


    def stop(self):
        self.motorsProxy.setV(0)
        self.motorsProxy.setW(0)