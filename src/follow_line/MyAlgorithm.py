#!/usr/bin/python
#-*- coding: utf-8 -*-

import numpy as np
import threading
import time
import cv2
from datetime import datetime

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
        imageLeft = self.cameraL.getImage().data
        imageRight = self.cameraR.getImage().data

        # Add your code here
        print "Runing"

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        #self.sensor.sendV(10)
        #self.sensor.sendW(5)

        #SHOW THE FILTERED IMAGE ON THE GUI
        self.setRightImageFiltered(imageRight)
        self.setLeftImageFiltered(imageLeft)


        # Add your code here
        print ("Runing")

        # RGB model change to HSV
        imageRight_HSV = cv2.cvtColor(imageRight, cv2.COLOR_RGB2HSV)
        imageLeft_HSV = cv2.cvtColor(imageLeft, cv2.COLOR_RGB2HSV)

        # Minimum and maximum values ​​of the red
        value_min_HSV = np.array([0, 235, 60])
        value_max_HSV = np.array([180, 255, 255])

        # Filtering images
        imageRight_HSV_filtered = cv2.inRange(imageRight_HSV, value_min_HSV, value_max_HSV)
        imageLeft_HSV_filtered = cv2.inRange(imageLeft_HSV, value_min_HSV, value_max_HSV)


        # Creating a mask with only the pixels within the range of red
        imageRight_HSV_filtered_Mask = np.dstack((imageRight_HSV_filtered, imageRight_HSV_filtered, imageRight_HSV_filtered))
        imageLeft_HSV_filtered_Mask = np.dstack((imageLeft_HSV_filtered, imageLeft_HSV_filtered, imageLeft_HSV_filtered))


        # Shape gives us the number of rows and columns of an image
        size = imageLeft.shape
        rows = size[0]
        columns = size[1]


        #  Looking for pixels that change of tone
        position_pixel_left = []
        position_pixel_right  = []

        for i in range(0, columns-1):
            value = imageLeft_HSV_filtered[365, i] - imageLeft_HSV_filtered[365, i-1]
            if(value != 0):
                if (value == 255):
                    position_pixel_left.append(i)
                else:
                    position_pixel_right.append(i-1)


        # Calculating the intermediate position of the road
        if ((len(position_pixel_left) != 0) and (len(position_pixel_right) != 0)):
            position_middle = (position_pixel_left[0] + position_pixel_right[0]) / 2
        elif ((len(position_pixel_left) != 0) and (len(position_pixel_right) == 0)):
            position_middle = (position_pixel_left[0] + columns) / 2
        elif ((len(position_pixel_left) == 0) and (len(position_pixel_right) != 0)):
            position_middle = (0 + position_pixel_right[0]) / 2
        else:
            position_pixel_right.append(1000)
            position_pixel_left.append(1000)
            position_middle = (position_pixel_left[0] + position_pixel_right[0])/ 2


        # Calculating the desviation
        desviation = position_middle - (columns/2)
        print (" desviation    ", desviation)

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        if (desviation == 0):
             self.motors.sendV(5)
        elif (position_pixel_right[0] == 1000):
             self.motors.sendW(-0.0000035)
        elif ((abs(desviation)) < 85):
             if ((abs(desviation)) < 15):
                 self.motors.sendV(1.5)
             else:
                 self.motors.sendV(3.5)
             self.motors.sendW(-0.000045 * desviation)
        elif ((abs(desviation)) < 150):
             if ((abs(desviation)) < 120):
                 self.motors.sendV(1)
             else:
                 self.motors.sendV(1.5)
             self.motors.sendW(-0.00045 * desviation)
        else:
             self.motors.sendV(1.5)
             self.motors.sendW(-0.0055 * desviation)


        #SHOW THE FILTERED IMAGE ON THE GUI
        self.setRightImageFiltered(imageRight_HSV_filtered_Mask)
        self.setLeftImageFiltered(imageLeft_HSV_filtered_Mask)
