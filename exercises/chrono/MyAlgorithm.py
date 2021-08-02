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
from decimal import Decimal, ROUND_HALF_EVEN

time_cycle = 80
#cursor = 0.0
sim_time = 0.0
initime = 0.0
posx = -59.25
posy = 28.6
bag = rosbag.Bag('2018-07-19-12-10-02.bag')
rep_duration = str(bag).split('duration:    ')[1].split()[1][1:-2]
rep_start = str(bag).split('start:       ')[1].split(' ')[4].split()[0][1:-1]

class MyAlgorithm(threading.Thread):

    def __init__(self, camera, motors, pose3d):
        self.camera = camera
        self.motors = motors
        self.pose = pose3d
        self.image=None
        self.carx = 0.0
        self.cary = 0.0
        self.cx = 0
        self.cy = 0
        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    def get_duration(self):
        rep_duration_min = int(rep_duration)/60
        rep_duration_seg = int(rep_duration)%60
        return str(rep_duration_min) + ":" + str(rep_duration_seg) + " min"

    def get_initime(self):
        return initime

    def synchronize(self):
        global posx, posy, cursor

        t_sim_unif = sim_time - initime + float(rep_start)
        if initime != 0.0 and t_sim_unif != 0.0:
            for (topic,msg,t) in bag.read_messages(start_time=rospy.Time(t_sim_unif-0.05)):
                t = t.to_sec()
                if t_sim_unif > t:
                    try:
                        posx = str(msg).split('x: ')[1].split()[0]
                        posy = str(msg).split('y: ')[1].split()[0]
                        return float(posx), float(posy)
                    except IndexError:
                        pass
                else:
                    return float(posx), float(posy)

        else:
            return float(posx), float(posy)

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
        global initime
        if self.is_alive():
            self.stop_event.clear()
        else:
            initime = rospy.Time.from_sec(rospy.get_time()).to_sec()
            self.start()
            #Start the recording
            print("Starting the record.")
            #rec = subprocess.Popen("rosbag record record rosout tf /F1ROS/odom /topic __name:=best_lap", shell=True)

    def kill (self):
        self.kill_event.set()
        print("Endind the record.")
        #rec_1 = subprocess.Popen("rosnode kill /best_lap", shell=True)

    def execute(self):
        global sim_time
        #GETTING THE IMAGES
        input_image = self.camera.getImage().data

        sim_time = rospy.Time.from_sec(rospy.get_time()).to_sec()

        if input_image is not None:
            #ADD YOUR CODE HERE

        #SHOW THE FILTERED IMAGE ON THE GUI
            self.setImageFiltered(image_HSV_filtered_Mask)
