#!/usr/bin/python
#-*- coding: utf-8 -*-
#
#  Copyright (C) 1997-2019 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#       Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>
#  Rosified by:
#       Francisco Perez Salgado <f.perez475@gmail.com>
#  Adapted to Turtlebot follow line by:
#       Julio Vega <julio.vega@urjc.es>

import threading
import time
from datetime import datetime
import math
import jderobot
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist     # Message that moves base

time_cycle = 80

class MyAlgorithm(threading.Thread):

    def __init__(self, camera, motors):
        self.twist = Twist()
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

    def algorithm(self): # Add your code here
        #EXAMPLE: GETTING IMAGES
        image = self.getImage()
        image2 = self.get_threshold_image()

        #EXAMPLE: PRINTING MESSAGES ON THE TERMINAL
        #print "Running"

        #EXAMPLE: SENDING COMMANDS TO THE MOTORS
        #self.motors.setV(10)
        #self.motors.setW(5)

        #EXAMPLE: SHOWING IMAGES ON THE GUI
        self.set_color_image(image)
        #self.set_threshold_image (image2)

        global perr, ptime, serr, dt

        img = cv2.resize(image,None,fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC)
        rows, cols, ch = img.shape
        pts1 = np.float32([[90,122],[313,122],[35,242],[385,242]])
        pts2 = np.float32([[0,0],[400,0],[0,400],[400,400]])
        M = cv2.getPerspectiveTransform(pts1,pts2)
        img_size = (img.shape[1], img.shape[0])
        image = cv2.warpPerspective(img,M,(img_size[0]+100,img_size[1]+100))#img_size
    
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        lower_yellow = np.array([ 10,  10,  10])
        upper_yellow = np.array([250, 255, 255])
        lower_white = np.array([200,100,100], dtype= "uint8")
        upper_white = np.array([255,255,255], dtype= "uint8")

        # Threshold to get only white
        maskw = cv2.inRange(image, lower_white, upper_white)
        masky = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Remove pixels not in this range
        mask_yw = cv2.bitwise_or(maskw, masky)    
        rgb_yw = cv2.bitwise_and(image, image, mask = mask_yw).astype(np.uint8)

        rgb_yw = cv2.cvtColor(rgb_yw, cv2.COLOR_RGB2GRAY)
 
        # Filter mask
        kernel = np.ones((7,7), np.uint8)
        opening = cv2.morphologyEx(rgb_yw, cv2.MORPH_OPEN, kernel)
        rgb_yw2 = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)		
	
        h, w= rgb_yw2.shape
        search_top = 7*h/8+20
        search_bot = 7*h/8 + 800 +20
        rgb_yw2[0:search_top, 0:w] = 0
        rgb_yw2[search_bot:h, 0:w] = 0
        M = cv2.moments(rgb_yw2)
        c_time = rospy.Time.now()
    
        if M['m00'] > 0:
           cxm = int(M['m10']/M['m00'])
           cym = int(M['m01']/M['m00'])
      
           cx = cxm - 110 #120#143 #CW
      
           if cxm <= 2*h/8:
      	      cx = cxm +(h/2)
      
           cv2.circle(rgb_yw2, (cxm, cym), 20, (255,0,0), -1)
           cv2.circle(rgb_yw2, (cx, cym), 20, (255,0,0),-1)
  
           # BEGIN CONTROL
           err = cx - 4*w/8
           self.twist.linear.x = 0.9
           dt = rospy.get_time() - ptime
           self.twist.angular.z = (-float(err) / 100)*2.5 + ((err - perr)/(rospy.get_time() - ptime))*1/50/100 #+ (serr*dt)*1/20/100 #1 is best, starting 3 unstable
           serr = err + serr
           perr = err
           ptime = rospy.get_time()

        else:
    	     self.twist.linear.x = 0.4
    	     self.twist.angular.z = -0.7
    	     err = 0

