import cv2
import time
import threading
import numpy as np
from datetime import datetime


time_cycle = 80

class MyAlgorithm(threading.Thread):

    def __init__(self, camera):
        self.camera = camera

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

        input_image = self.camera.getImage()
        if input_image is not None:
            self.camera.setColorImage(input_image)
            
            """
            If you want show a threshold image (black and white image)
            self.camera.setThresholdImage(bw_image)
            
            If you want to show the detected image
            self.camera.setDetectImage(detect_image)

            If you want to access the current value of slider 
            for example say in RGB space if you want minimum value set of R scale use `self.camera.RGB_R_min_out`
            
            If you want to check the color space selected use `self.camera.currDropdown`
            """
