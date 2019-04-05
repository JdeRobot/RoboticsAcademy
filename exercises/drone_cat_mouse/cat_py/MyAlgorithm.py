import threading
import time
from datetime import datetime
import signal
import sys

time_cycle = 80

def nothing(x):
	pass

class MyAlgorithm(threading.Thread):

    def __init__(self, drone):
        self.drone = drone

        self.drone.sendCMDVel(0, 0, 0, 0, 0, 0)
    	self.drone.takeoff()

        self.image = None

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    def setImageFiltered(self, image):
        self.lock.acquire()
        self.image=image
        self.lock.release()

    def getImageFiltered(self):
        self.lock.acquire()
        tempImage=self.image
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
	#Add your code here
	
	#To get the camera images
    	droneImage = self.drone.getImage().data
	
	#Te set the filtered images on the GUI
	#self.setImageFiltered(image_filtered)
