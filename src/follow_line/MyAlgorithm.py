#from sensors import sensor
import numpy as np
import threading


class MyAlgorithm():

    def __init__(self, sensor):
        self.sensor = sensor
        self.imageRight=None
        self.imageLeft=None
        self.lock = threading.Lock()



    def execute(self):
        #GETTING THE IMAGES
        imageLeft = self.sensor.getImageLeft()
        imageRight = self.sensor.getImageRight()


        # Add your code here
        print "Runing"

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        #self.sensor.setV(10)
        #self.sensor.setW(5)


        #SHOW THE FILTERED IMAGE ON THE GUI
        self.setRightImageFiltered(imageRight)
        self.setLeftImageFiltered(imageLeft)



    def setRightImageFiltered(self, image):
        self.lock.acquire()
        self.imageRight=image
        self.lock.release()


    def setLeftImageFiltered(self, image):
        self.lock.acquire()
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

