#!/usr/bin/python
#-*- coding: utf-8 -*-

import threading
import time
from datetime import datetime
import cv2
import numpy as np

from sensors.cameraFilter import CameraFilter
from parallelIce.navDataClient import NavDataClient
from parallelIce.cmdvel import CMDVel
from parallelIce.extra import Extra
from parallelIce.pose3dClient import Pose3DClient


time_cycle = 80

class MyAlgorithm(threading.Thread):


    class pid(object):
        def __init__(self, kp, kd, ki):
            # Constant of PID control
            self.kp = kp
            self.kd = kd
            self.ki = ki
            self.error = 0
            self.acumulate_error = 0

        def calculateU(self, e):
            proportional = self.kp * e
            derivate = self.kd * (e - self.error)
            self.acumulate_error = self.acumulate_error + e
            integral = self.ki*(self.acumulate_error)
            u =  -(proportional) -(derivate) -(integral)
            self.error = e
            return u


    def __init__(self, camera, navdata, pose, cmdvel, extra):
        self.camera = camera
        self.navdata = navdata
        self.pose = pose
        self.cmdvel = cmdvel
        self.extra = extra

        self.minError = 0.013

        # Constructor PID
        self.pidX = self.pid(2.655,0.000112,0.00029)
        self.pidY = self.pid(2.655,0.000112,0.00029)

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    def run (self):

        self.stop_event.clear()

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


    def turtlebot_visual_field(self, contours):
        return bool(contours)

    def bounding(self, contours, colors, input_image_Copy):
        area = 0
        if (len(contours) != 0):
            cnt = cv2.approxPolyDP(contours[0], 3, True)
            rectX, rectY, rectW, rectH = cv2.boundingRect(cnt)

        for cnt in contours:
            # Approximates a polygonal curve(s) with the specified precision.
            cnt = cv2.approxPolyDP(cnt, 3, True);
            # It is a straight rectangle, it doesn't consider the rotation of the object. So area of the bounding rectangle won't be minimum.
            # Let (x,y) be the top-left coordinate of the rectangle and (w,h) be its width and height.
            x, y, w, h= cv2.boundingRect(cnt)
            if((x - w) * (y -h) > area):
                rectX = x
                rectY = y
                rectH = h
                rectW = w
            if (len(contours) != 0):
                cv2.rectangle(input_image_Copy, (rectX, rectY), (rectX+rectW,rectY+rectH), (colors[0],colors[1], colors[2]), 2)
            center = [rectX+(rectW/2), rectY+(rectH/2)]
            return center

    def execute(self):
       # Add your code here

        input_image = self.camera.getImage()
        if input_image is not None:
            self.camera.setColorImage(input_image)
            '''
            If you want show a thresold image (black and white image)
            self.camera.setThresoldImage(input_image)
            '''

            # Gaussian filter (Soften the image)
            gaussian_image = cv2.GaussianBlur(input_image, (5,5), 0.2)

            # RGB model change to HSV
            hsv_image = cv2.cvtColor(gaussian_image, cv2.COLOR_RGB2HSV)

            # Minimum and maximum values ​​of the green
            value_min_HSV = np.array([50, 21, 11])#52.78, 130.05,131.94
            value_max_HSV = np.array([72, 235, 244])#114.5,255, 255

            # Filtering image
            image_HSV_filtered = cv2.inRange(hsv_image, value_min_HSV, value_max_HSV)

            # Close, morphology element
            kernel = np.ones((19,19), np.uint8)
            image_HSV_filt_close = cv2.morphologyEx(image_HSV_filtered, cv2.MORPH_CLOSE, kernel)

            # Output image
            self.camera.setThresoldImage(image_HSV_filt_close)

            # Copying image
            image_HSV_filtered_Copy = np.copy(image_HSV_filt_close)
            input_image_Copy = np.copy(input_image)

            # Detection of object contour
            _, contours, hierarchy = cv2.findContours(image_HSV_filtered_Copy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Size of image
            size = input_image.shape
            height = size[0]
            width = size[1]

            # Image center
            img_center = [width/2, height/2]

            # Bounding box
            color = [255, 0, 0]
            center = self.bounding(contours, color, input_image_Copy)

            # Output image
            self.camera.setColorImage(input_image_Copy)

            if self.turtlebot_visual_field(contours):
                # Calculating the error
                errorx = (center[0] - img_center[0])/320
                errory = (center[1] - img_center[1])/320

                # Controlador PID
                controladorX = self.pidX.calculateU(errorx)
                controladorY = self.pidY.calculateU(errory)
                print ("error",errorx, errory)
                print ("controlador", controladorX, controladorY)

                # Correct position
                if (abs(errorx) <= self.minError) and (abs(errory) >= self.minError):
                    self.cmdvel.sendCMDVel(0.5*controladorY,0,0,0,0,0)
                if (abs(errorx) >= self.minError) and (abs(errory) <= self.minError):
                    self.cmdvel.sendCMDVel(0,0.5*controladorX,0,0,0,0)
                if (abs(errorx) >= self.minError) and (abs(errory) >= self.minError):
                    self.cmdvel.sendCMDVel(0.5*controladorY,0.5*controladorX,0,0,0,0)
                elif (abs(errorx) <= self.minError) and (abs(errory) <= self.minError):
                    # If the margin is minimum, then we have got the target
                    # The acumulate error is zero
                    self.pidX.acumulate_error = 0
                    self.pidY.acumulate_error = 0
                    self.cmdvel.sendCMDVel(0,0,0,0,0,0)

            else:
                # Climb up to see the turtlebot
                self.cmdvel.sendCMDVel(0,0,0.2,0,0,0)
                pass
