import threading
import time
from datetime import datetime
import cv2
import numpy as np

time_cycle = 80


class MyAlgorithm(threading.Thread):

    def __init__(self, camera, navdata, pose, cmdvel, extra):
        self.camera = camera
        self.navdata = navdata
        self.pose = pose
        self.cmdvel = cmdvel
        self.extra = extra
        self.input_image=None
        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    def setImageFiltered(self, image):
        self.lock.acquire()
        self.input_image=image
        self.lock.release()

    def getImageFiltered(self):
        self.lock.acquire()
        tempImage=self.input_image
        self.lock.release()
        return tempImage

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
        input_image = self.camera.getImage().data

        # Show the Filtered_Image on the GUI
        self.setImageFiltered(input_image)

        # ADD YOUR CODE HERE
