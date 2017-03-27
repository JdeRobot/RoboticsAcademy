import numpy as np
import threading
import time
from datetime import datetime

from jderobotTypes import CMDVel
from jderobotTypes import Image

time_cycle = 80

class MyAlgorithm(threading.Thread):

    def __init__(self, cameraL, cameraR, motors):
        self.cameraL = cameraL
        self.cameraR = cameraR
        self.motors = motors
        self.imageRight=None
        self.imageLeft=None
        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

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

    def run (self):

        while (not self.kill_event.is_set()):
           
            start_time = datetime.now()

            if not self.stop_event.is_set():
                self.execute()

            finish_Time = datetime.now()

            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            #print (ms)
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)

    def stop (self):
        self.stop_event.set()

    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()

    def kill (self):
        self.kill_event.set()


    def execute(self):
        #GETTING THE IMAGES
        imageLeft = self.cameraL.getImage()
        imageRight = self.cameraR.getImage()


        # Add your code here
        print "Running"

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        #self.motors.setV(10)
        #self.motors.setW(5)
        #self.motors.sendVelocities()


        #SHOW THE FILTERED IMAGE ON THE GUI
        self.setRightImageFiltered(imageRight.data)
        self.setLeftImageFiltered(imageLeft.data)

