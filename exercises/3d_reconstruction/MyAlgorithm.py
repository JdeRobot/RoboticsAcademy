#!/usr/bin/python
#-*- coding: utf-8 -*-
import threading
import time
import sys
from datetime import datetime

import math
import cv2
import numpy as np

from interfaces.camera import ListenerParameters

time_cycle = 80

class MyAlgorithm(threading.Thread):

    def __init__(self, cameraL, cameraR, point):
        self.cameraL = cameraL
        self.cameraR = cameraR
        self.point = point
        self.threshold_image = np.zeros((640,360,3), np.uint8)
        self.color_image = np.zeros((640,360,3), np.uint8)
        
        print("Left Camera Configuration File:")
        self.camLeftP = ListenerParameters(sys.argv[1], "CamACalibration")
        print("Right Camera Configuration File:")
        self.camRightP = ListenerParameters(sys.argv[1], "CamBCalibration")
        
        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        self.threshold_image_lock = threading.Lock()
        self.color_image_lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)
    
    def getImage(self, lr):
        self.lock.acquire()
        if(lr == 'left'):
        	img = self.cameraL.getImage().data
        elif(lr == 'right'):
        	img = self.cameraR.getImage().data
        else:
        	print("Invalid camera")
        	exit()
        self.lock.release()
        return img

    def run (self):
	self.algorithm()

    def stop (self):
        self.stop_event.set()

    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()

    def kill (self):
        self.kill_event.set()

    def algorithm(self):
        #GETTING THE IMAGES
        image = self.getImage('right')
        #print(image.shape)
        #cv2.imshow('a', image)
	for i in range(3):
		self.point.plotPoint([1.0, 0.0, i], [1.0, 0.0, 0.0])
		time.sleep(5)
        # Add your code here
        print "Runing"

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        #self.motors.sendV(10)
        #self.motors.sendW(5)

        #SHOW THE FILTERED IMAGE ON THE GUI
       	#self.set_threshold_image(image)
