import threading
import time
from datetime import datetime
import cv2
import numpy as np
import math

time_cycle = 80

class MyAlgorithm(threading.Thread):

    def __init__(self, drone):
        self.drone = drone

        self.height = 240
        self.width = 320

        self.imageV=None
        self.imageF =None

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    def setImageFilteredVentral(self, image):
        self.lock.acquire()
        self.imageV=image
        self.lock.release()

    def getImageFilteredVentral(self):
        self.lock.acquire()
        tempImageV=self.imageV
        self.lock.release()
        return tempImageV

    def setImageFilteredFrontal(self, image):
        self.lock.acquire()
        self.imageF=image
        self.lock.release()

    def getImageFilteredFrontal(self):
        self.lock.acquire()
        tempImageF=self.imageF
        self.lock.release()
        return tempImageF

    def run (self):

        self.stop_event.clear()

        while (not self.kill_event.is_set()):

            start_time = datetime.now()

            if not self.stop_event.is_set():
                self.execute()

            finish_Time = datetime.now()

            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
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
        input_imageV = self.drone.getImageVentral().data
        input_imageF = self.drone.getImageFrontal().data

        if input_imageV is not None:
	    #printing the filtered image
            self.setImageFilteredVentral(image_HSV_filtered_Mask_V)

        if input_imageF is not None:
            #printing the filtered image
            self.setImageFilteredFrontal(image_HSV_filtered_Mask_F)
