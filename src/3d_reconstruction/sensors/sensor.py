import sys, traceback, Ice
import easyiceconfig as EasyIce
import comm, config
import jderobot
import numpy as np
import threading
import pointBuffer
import cv2


class Sensor:
    def __init__(self):
        self.lock = threading.Lock()
        self.playButton=False

        try:
            cfg = config.load(sys.argv[1])
            #starting comm
            jdrc= comm.init(cfg, '3DReconstruction')

            ic = jdrc.getIc()
            properties = ic.getProperties()

            proxyStrCL = jdrc.getConfig().getProperty("3DReconstruction.CameraLeft.Proxy")
            basecameraL = ic.stringToProxy(proxyStrCL)
            #ic = EasyIce.initialize(sys.argv)
            #properties = ic.getProperties()
            #basecameraL = ic.propertyToProxy("FollowLine.CameraLeft.Proxy")
            self.cameraProxyL = jderobot.CameraPrx.checkedCast(basecameraL)

            if self.cameraProxyL:
                self.imageLeft = self.cameraProxyL.getImageData("RGB8")
                self.imageLeft_h= self.imageLeft.description.height
                self.imageLeft_w = self.imageLeft.description.width
            else:
                print ('Interface for left camera not connected')

            proxyStrCR = jdrc.getConfig().getProperty("3DReconstruction.CameraRight.Proxy")
            basecameraR = ic.stringToProxy(proxyStrCL)
            #basecameraR = ic.propertyToProxy("FollowLine.CameraRight.Proxy")
            self.cameraProxyR = jderobot.CameraPrx.checkedCast(basecameraR)

            if self.cameraProxyR:
                self.imageRight = self.cameraProxyR.getImageData("RGB8")
                self.imageRight_h= self.imageRight.description.height
                self.imageRight_w = self.imageRight.description.width
            else:
                print ('Interface for right camera not connected')


            proxyStrM = jdrc.getConfig().getProperty("3DReconstruction.Motors.Proxy")
            motorsBase = ic.stringToProxy(proxyStrM)
            #motorsBase = ic.propertyToProxy("FolowLine.motors.Proxy")
            self.motorsProxy = jderobot.MotorsPrx.checkedCast(motorsBase)
            if self.motorsProxy:
                print ('Interface for motors connected')
            else:
                print ('Interface for motors not connected')


            self.maxSpeedV=1
            self.maxSpeedW=1


            #visualization
        #    proxyStrV = jdrc.getConfig().getProperty("3DReconstruction.Viewer.Proxy")
        #    baseViewer = ic.stringToProxy(proxyStrV)
            #baseViewer = ic.propertyToProxy("FollowLine.Viewer.Proxy")
        #    self.viewerProxy = jderobot.VisualizationPrx.checkedCast(baseViewer)
            #if self.viewerProxy:
        #        print ('Interface for viewer connected')
        #    else:
        #        print ('Interface for viewer not connected')

            #draw floor:
            self.MAXWORLD=30
            colorJDE= jderobot.Color()
            colorJDE.r=0
            colorJDE.g=0
            colorJDE.b=0
            for i in range(0,self.MAXWORLD+1):
                pointJde1= jderobot.Point()
                pointJde2= jderobot.Point()
                pointJde3= jderobot.Point()
                pointJde4= jderobot.Point()
                pointJde1.x=-self.MAXWORLD * 10 / 2 + i * 10
                pointJde1.y= -self.MAXWORLD * 10 / 2
                pointJde1.z=0
                pointJde2.x=-self.MAXWORLD * 10 / 2 + i * 10
                pointJde2.y= self.MAXWORLD * 10 / 2
                pointJde2.z=0
                pointJde3.x=-self.MAXWORLD * 10 / 2
                pointJde3.y=-self.MAXWORLD * 10 / 2. + i * 10
                pointJde3.z=0
                pointJde4.x=self.MAXWORLD * 10 / 2
                pointJde4.y=-self.MAXWORLD * 10 / 2. + i * 10
                pointJde4.z=0
                seg1= jderobot.Segment()
                seg1.fromPoint=pointJde1
                seg1.toPoint=pointJde2
                seg2= jderobot.Segment()
                seg2.fromPoint=pointJde3
                seg2.toPoint=pointJde4
                pointBuffer.getbufferSegment(seg1, colorJDE)
                #self.viewerProxy.drawSegment(seg1,colorJDE)
                pointBuffer.getbufferSegment(seg2, colorJDE)
                #self.viewerProxy.drawSegment(seg2,colorJDE)




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
            imgL=cv2.resize(imgL,(320,240))
            return imgL;

        return None

    def getImageRight(self):
        if self.cameraProxyR:
            self.lock.acquire()
            imgR = np.zeros((self.imageRight_h, self.imageRight_w, 3), np.uint8)
            imgR = np.frombuffer(self.imageRight.pixelData, dtype=np.uint8)
            imgR.shape = self.imageRight_h, self.imageRight_w, 3
            self.lock.release()
            imgR=cv2.resize(imgR,(320,240))
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

    def drawPoint(self,point, color=(0,0,0)):


        pointJde= jderobot.Point()
        pointJde.x=float(point[0])/100.0
        pointJde.y=float(point[1]/100.0)
        pointJde.z=float(point[2]/100.0)


        colorJDE= jderobot.Color()
        colorJDE.r=float(color[0])
        colorJDE.g=float(color[1])
        colorJDE.b=float(color[2])
        pointBuffer.getbufferPoint(pointJde, colorJDE)
        #self.viewerProxy.drawPoint(pointJde,colorJDE)
