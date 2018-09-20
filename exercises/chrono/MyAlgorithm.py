#!/usr/bin/python
#-*- coding: utf-8 -*-

import numpy as np
import threading
import time
import cv2
from datetime import datetime
import subprocess
import rospy
import time
import rosbag

time_cycle = 80
sync_time = 0.0
sim_time = 0.0
initime = 0.0
posx = -58.6536031326
posy = 28.2358310717

class MyAlgorithm(threading.Thread):

    def __init__(self, camera, motors, pose3d):
        self.camera = camera
        self.motors = motors
        self.pose = pose3d
        self.image=None
        self.carx = 0.0
        self.cary = 0.0
        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    def synchronize(self):
        global posx, posy, sync_time

        real_time = sim_time - initime
        if initime != 0.0 :
            bag = rosbag.Bag('2018-08-24-12-48-46.bag')
            for (topic,msg,t) in bag.read_messages():
                try:
                    posx = str(msg).split('x: ')[1].split()[0]
                    posy = str(msg).split('y: ')[1].split()[0]

                    t = float(str(t)) * 1e-9

                    print(t,real_time)
                    #print(str(sync_time) + ": " + str(t) + " || " + str(real_time) + ";; " + str(sim_time) + ";; " + str(initime))

                    if sync_time < t:
                        sync_time = t
                        if real_time < t:
                            return float(posx), float(posy)

                except IndexError:
                    pass

        else:
            return posx, posy

    def setImageFiltered(self, image):
        self.lock.acquire()
        self.image=image
        self.lock.release()

    def getImageFiltered(self):
        self.lock.acquire()
        tempImage=self.image
        self.lock.release()
        return tempImage

    def getCarDirection(self):
        self.carx = pose.getPose().x
        self.cary = pose.getPose().y
        return (self.carx, self.cary)

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
        self.stop_event.set()
        self.motors.sendV(0)
        self.motors.sendW(0)

    def play (self):
        global initime, bag

        if self.is_alive():
            self.stop_event.clear()
        else:
            initime = float(rospy.get_time())
            self.start()
            #Start the recording
            #rec = subprocess.Popen("rosbag record record rosout tf /F1ROS/odom /topic __name:=best_lap", shell=True)
            #rep = subprocess.Popen("rosbag play -r 0.10 2018-08-24-12-48-46.bag /F1ROS/odom:=/F1ROS_phantom/odom", shell=True)
            #rep = subprocess.Popen("rosbag play 2018-08-24-12-48-46.bag /F1ROS/odom:=/F1ROS_phantom/odom", shell=True)

    def kill (self):
        self.kill_event.set()
        #rec_1 = subprocess.Popen("rosnode kill /best_lap", shell=True)
        #rep_1 = subprocess.Popen("rosnode list | grep /play_* | xargs rosnode kill")

    def execute(self):
        global sim_time
        #GETTING THE IMAGES
        image = self.camera.getImage().data


        # Add your code here
        #print "Runing"

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        #self.sensor.sendV(10)
        #self.sensor.sendW(5)

        #SHOW THE FILTERED IMAGE ON THE GUI
        #self.setImageFiltered(image)

        # RGB model change to HSV
        image_HSV = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        # Minimum and maximum values ​​of the red
        value_min_HSV = np.array([0, 100, 0])
        value_max_HSV = np.array([180, 255, 255])

        # Filtering images
        image_HSV_filtered = cv2.inRange(image_HSV, value_min_HSV, value_max_HSV)


        # Creating a mask with only the pixels within the range of red
        image_HSV_filtered_Mask = np.dstack((image_HSV_filtered, image_HSV_filtered, image_HSV_filtered))


        # Shape gives us the number of rows and columns of an image
        size = image.shape
        rows = size[0]
        columns = size[1]


        #  Looking for pixels that change of tone
        position_pixel_left = []
        position_pixel_right  = []

        for i in range(0, columns-1):
            value = image_HSV_filtered[365, i] - image_HSV_filtered[365, i-1]
            if(value != 0):
                if (value == 255):
                    position_pixel_left.append(i)
                else:
                    position_pixel_right.append(i-1)


        # Calculating the intermediate position of the road
        if ((len(position_pixel_left) != 0) and (len(position_pixel_right) != 0)):
            position_middle = (position_pixel_left[0] + position_pixel_right[0]) / 2
        elif ((len(position_pixel_left) != 0) and (len(position_pixel_right) == 0)):
            position_middle = (position_pixel_left[0] + columns) / 2
        elif ((len(position_pixel_left) == 0) and (len(position_pixel_right) != 0)):
            position_middle = (0 + position_pixel_right[0]) / 2
        else:
            position_pixel_right.append(1000)
            position_pixel_left.append(1000)
            position_middle = (position_pixel_left[0] + position_pixel_right[0])/ 2


        # Calculating the desviation
        desviation = position_middle - (columns/2)
        #print (" desviation    ", desviation)

        sim_time = float(self.pose.getPose3d().timeStamp)

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        if (desviation == 0):
             self.motors.sendV(3)
        elif (position_pixel_right[0] == 1000):
             self.motors.sendW(-0.0000035)
        elif ((abs(desviation)) < 85):
             if ((abs(desviation)) < 31):
                 self.motors.sendV(3)
             else:
                 self.motors.sendV(1)
             self.motors.sendW(-0.000045 * desviation)
        elif ((abs(desviation)) < 150):
             if ((abs(desviation)) < 120):
                 self.motors.sendV(1)
             else:
                 self.motors.sendV(1)
             self.motors.sendW(-0.00045 * desviation)
        else:
             self.motors.sendV(1)
             self.motors.sendW(-0.0055 * desviation)



        #SHOW THE FILTERED IMAGE ON THE GUI
        self.setImageFiltered(image_HSV_filtered_Mask)
