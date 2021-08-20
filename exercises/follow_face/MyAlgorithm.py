# -*- coding: utf-8 -*-

import threading
import time
from datetime import datetime
import cv2
import numpy as np


time_cycle = 80

class MyAlgorithm(threading.Thread):

    def __init__(self, camera, motors):
        self.camera = camera
        self.motors = motors

        self.pan = 0.0
        self.tilt = 0.0
        self.first_time = True
        self.time = 0.0
        self.topright = False
        # MOVE CAMERA TO A KNOWN POSITION
        if (self.motors):
            self.motors.setPTMotorsData(0, 0, 24, 20)

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


    def execute(self):
        # Add your code here
        # Input image
        input_image = self.camera.getImage()
        if input_image is not None:
            self.camera.setColorImage(input_image)
            '''
            If you want show a thresohld image (black and white image) or a gray 
            or filtered image use:
            self.camera.setThresholdImage(image)
            '''
