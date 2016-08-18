from sensors import sensor
import numpy as np
import threading
from pyProgeo.progeo import Progeo
import cv2


class MyAlgorithm():

    def __init__(self, sensor):
        self.sensor = sensor
        self.imageRight=None
        self.imageLeft=None
        self.lock = threading.Lock()
        self.camLeftP=Progeo("cameras/camA.json")
        self.camRightP=Progeo("cameras/camB.json")
        self.done=False

    def execute(self):
        #GETTING THE IMAGES
        imageLeft = self.sensor.getImageLeft()
        imageRight = self.sensor.getImageRight()

        if self.done:
            return

        # Add your code here
        # pointIn=np.array([502,21,1])
        # pointInOpt=self.camLeftP.graficToOptical(pointIn)
        # point3d=self.camLeftP.backproject(pointInOpt)
        # projected1 = self.camRightP.project(point3d)
        # print self.camRightP.opticalToGrafic(projected1)





        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        #self.sensor.setV(10)
        #self.sensor.setW(5)


        #SHOW THE FILTERED IMAGE ON THE GUI
        # self.setRightImageFiltered(imageRight)
        # self.setLeftImageFiltered(imageLeft)

        #PLOT 3D data on the viewer
        #point=np.array([1, 1, 1])
        #self.sensor.drawPoint(point,(255,255,255))



    def setRightImageFiltered(self, image):
        self.lock.acquire()
        size=image.shape
        if len(size) == 2:
            image=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
        self.imageRight=image
        self.lock.release()


    def setLeftImageFiltered(self, image):
        self.lock.acquire()
        size=image.shape
        if len(size) == 2:
            image=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
        self.imageLeft=image
        self.lock.release()

    def getRightImageFiltered(self):
        self.lock.acquire()
        tempImage=self.imageRight
        self.lock.release()
        return tempImage

    def getLeftImageFiltered(self):
        self.lock.acquire()
        tempImage=self.imageLeft
        self.lock.release()
        return tempImage
