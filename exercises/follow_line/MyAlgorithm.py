#!/usr/bin/python
#-*- coding: utf-8 -*-
import threading
import time
from datetime import datetime

import math
import jderobot
import cv2
import numpy as np

time_cycle = 80

class MyAlgorithm(threading.Thread):

    def __init__(self, camera, motors):
        self.camera = camera
        self.motors = motors
        self.threshold_image = np.zeros((640,360,3), np.uint8)
        self.color_image = np.zeros((640,360,3), np.uint8)
        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        self.threshold_image_lock = threading.Lock()
        self.color_image_lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)
    
    def getImage(self):
        self.lock.acquire()
        img = self.camera.getImage().data
        self.lock.release()
        return img

    def set_color_image (self, image):
        img  = np.copy(image)
        if len(img.shape) == 2:
          img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        self.color_image_lock.acquire()
        self.color_image = img
        self.color_image_lock.release()
        
    def get_color_image (self):
        self.color_image_lock.acquire()
        img = np.copy(self.color_image)
        self.color_image_lock.release()
        return img
        
    def set_threshold_image (self, image):
        img = np.copy(image)
        if len(img.shape) == 2:
          img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        self.threshold_image_lock.acquire()
        self.threshold_image = img
        self.threshold_image_lock.release()
        
    def get_threshold_image (self):
        self.threshold_image_lock.acquire()
        img  = np.copy(self.threshold_image)
        self.threshold_image_lock.release()
        return img

    def run (self):

        while (not self.kill_event.is_set()):
            start_time = datetime.now()
            if not self.stop_event.is_set():
                self.algorithm()
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

    def algorithm(self):
        #GETTING THE IMAGES
        img= self.getImage()
        
            # Add your code here
           
	    height, width, channels = img.shape
        kernel = np.ones((5,5),np.uint8)	
        
        image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #defining range for mask1
        lower_red = np.array([0,100,20],np.uint8) 
        upper_red = np.array([10,255,255],np.uint8)

        #defining range for mask2
        lower_red1 = np.array([170,100,20],np.uint8)
        upper_red1 = np.array([180,255,255],np.uint8)

        mask1 = cv2.inRange(hsv, lower_red,upper_red)
        mask2 = cv2.inRange(hsv, lower_red1,upper_red1)

        mask = mask1|mask2
        #Edge detection approach
    #	edges = cv2.Canny(mask, 75, 150)
    #	lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=50)
    #        if lines is not None:
    #	    for line in lines:
    #	        x1, y1, x2, y2 = line[0]
    #	        cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 5)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_or(image,image, mask= mask)
    #	opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
        # Drawing centroid in the resultant image
        cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)

        #Error/difference between line to be followed and current path for proportional controller
        error_x = cx - width / 2;

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        self.motors.sendV(5)
        self.motors.sendW(-error_x / 100)
        print "Runing"
        #SHOW THE FILTERED IMAGE ON THE GUI
        self.set_threshold_image(res)        
       
