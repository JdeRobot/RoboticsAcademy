import numpy as np
import cv2
import math

import sys

import threading
import time
from datetime import datetime
import yaml
import rospy
from std_msgs.msg import Float32
from interfaces.moveBaseClient import clearCostmaps

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

    def __init__(self, grid, sensor, vel, pathListener, moveBaseClient):
        self.sensor = sensor
        self.grid = grid
        self.vel = vel
        self.path = pathListener
        sensor.getPathSig.connect(self.sendGoal)
        # Coordinates for all pallets in a map in world frame. Please use these to approximate center of pallet.
        self.palettesList = yaml.load(open('./pallets_coords.yaml'))["coords"]
        self.jointForce = 0
        self.pub = rospy.Publisher('amazon_warehouse_robot/joint_cmd', Float32, queue_size=10)
        self.client = moveBaseClient
        self.pickNewPalletPressed = False

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

    def liftDropExecute(self):
        if self.jointForce != 25:
            self.jointForce = 25
            self.pub.publish(self.jointForce)
            print ('Platform Lifted!')
        else:
            self.jointForce = 0
            self.pub.publish(self.jointForce)
            print ('Platform Dropped!')

    """ Write in this method the code for drawing path. """
    def drawPath(self):
        print("Drawing path")
        #  Use self.path.getPath()

    """ This method will set pickNewPalletPressed == True when the Store 
        New Pallet button is pressed. """
    def setNewPalletFlag(self, isPressed):
        print("Pick new pallet button pressed")
        self.pickNewPalletPressed = isPressed

    """ Write in this method the code necessary to go to the desired pallet.
        The destiny is chosen in the GUI, making double click in the map.
        This method will be call when you press the Send Goal button. """
    def sendGoal(self, list):
        print("EXECUTING GOAL")
        # Get destination after double click on map
        # destiny = self.grid.getDestiny()

        # HOW TO SEND GOAL TO A ROBOT
        # Before sending goal to the robot, don't forget to 
        # change it to the world frame
        dest = self.grid.gridToWorld(24, 151)
        self.client.sendGoalToClient(dest[0], dest[1])

    """ Write in this method the code necessary for controlling the main 
        functionality of the robot. This method will be periodically 
        called after you press the GO! button. """
    def execute(self):
        # Add your code here
        print("Running")

        # LIFT PALLET
        self.liftDropExecute()

        # TO DO
