#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import threading
import time
from datetime import datetime
import jderobot
import math
from math import pi as pi
import cv2

from matplotlib import pyplot as plt
time_cycle = 80


class MyAlgorithm(threading.Thread):

    def __init__(self, pose3d, cameraC, cameraL, cameraR, motors):
        self.cameraL = cameraL
        self.cameraR = cameraR
        self.cameraC = cameraC
        self.pose3d = pose3d
        self.motors = motors

        # 0 to grayscale
        self.template = cv2.imread('resources/template.png',0)

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)


    def setImageFiltered(self, image):
        self.lock.acquire()
        self.lock.release()


    def getImageFiltered(self):
        self.lock.acquire()
        tempImage=self.image
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
        self.motors.sendV(0)
        self.motors.sendW(0)
        self.stop_event.set()


    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()


    def kill (self):
        self.kill_event.set()

    def execute(self):

        # TODO

        # Getting the images
        input_image = self.cameraC.getImage().data
