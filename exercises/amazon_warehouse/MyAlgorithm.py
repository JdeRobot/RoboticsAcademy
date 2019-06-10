import numpy as np
import cv2
import math

import threading
import time
from datetime import datetime

time_cycle = 80

def clearscreen(numlines=10):
    """Clear the console.
    numlines is an optional argument used only as a fall-back.
    """
    import os
    if os.name == "posix":
        # Unix/Linux/MacOS/BSD/etc
        os.system('clear')
    elif os.name in ("nt", "dos", "ce"):
        # DOS/Windows
        os.system('CLS')
    else:
        # Fallback for other operating systems.
        print '\n' * numlines

class MyAlgorithm(threading.Thread):

    def __init__(self, grid, sensor, vel, laser):
        self.sensor = sensor
        self.grid = grid
        self.vel = vel
        self.laser = laser
        sensor.getPathSig.connect(self.generatePath)

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

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
            self.dt = ms/1000

    def stop (self):
        self.stop_event.set()

    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()

    def kill (self):
        self.kill_event.set()

    """ Write in this method the code necessary for looking for the shorter
        path to the desired destiny.
        The destiny is chosen in the GUI, making double click in the map.
        This method will be call when you press the Generate Path button.
        Call to grid.setPath(path) method for setting the path. """
    def generatePath(self, list):
        print("LOOKING FOR SHORTER PATH")
        mapIm = self.grid.getMap()
        dest = self.grid.getDestiny()
        gridPos = self.grid.getPose()

        # TODO

        #Represent the Gradient Field in a window using cv2.imshow
        self.grid.showGrid()


    """ Write in this method the code necessary for going to the desired place,
        once you have generated the shorter path.
        This method will be periodically called after you press the GO! button. """
    def execute(self):
	    # Add your code here
        print("Running")

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        self.vel.setV(0.2)
        self.vel.setW(0.1)

        #self.sensor.getRobotTheta()
	    #self.sensor.getRobotY()

	    # TODO
