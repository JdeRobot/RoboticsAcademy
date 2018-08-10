import numpy as np
import threading
import time
from datetime import datetime
import jderobot
import math
import cv2
from math import pi as pi
from navigation.navigation import navigation
from navigation.laser.laser_handle import laser_handle
import matplotlib.pyplot as plt
from scipy import interpolate  

time_cycle = 80


class MyAlgorithm(threading.Thread):

    def __init__(self, pose3d, laser1, laser2, laser3, motors):
        self.pose3d = pose3d
        self.laser1 = laser1
        self.laser2 = laser2
        self.laser3 = laser3
        self.motors = motors

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

        self.navigation = navigation(None,0, 0, 1.25)
        #'/home/hywel/JdeRobot_ws/colab-gsoc2018-HanqingXie/AutoparkOmpl/init_map.ppm'

        self.find_path = False
        self.find_target = False
        self.laserHandle = laser_handle()
        

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


    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()


    def kill (self):
        self.kill_event.set()
            
            
    def execute(self):
        pose = [0,0,0]
        pose[0] = self.pose3d.getPose3d().x
        pose[1] = self.pose3d.getPose3d().y
        pose[2] = self.pose3d.getPose3d().yaw

        if not self.find_target:
            laser2_data = self.laser3.getLaserData()
            laser2 = self.parse_laser_data(laser2_data)
            target = self.laserHandle.autopark_place(laser2, pose)
            if target:
                self.find_target = True
                self.target = target
                self.motors.sendV(0)
                self.motors.sendW(0)
                print ("target", target)
            else:
                self.motors.sendV(5)

        if not self.find_path and self.find_target:
            self.motors.sendV(0)
            self.motors.sendW(0)
            #car (6,3)
            goal_pose = [0,0,0]
            goal_pose[0] = self.target[0]
            goal_pose[1] = self.target[1]
            goal_pose[2] = self.target[2]
            self.navigation.update_map_autopark(pose,goal_pose)
            self.find_path = self.navigation.path_planning(pose,goal_pose)

        if self.find_path and self.find_target:
            data = self.navigation.path_following(pose)
            self.motors.sendV(data[0])
            self.motors.sendW(data[1])

            

        '''
        laser0_data = self.laser1.getLaserData()
        laser0 = self.parse_laser_data(laser0_data)
        self.navigation.updateMap(laser0,0,pose)

        laser1_data = self.laser2.getLaserData()
        laser1 = self.parse_laser_data(laser1_data)
        self.navigation.updateMap(laser1,1,pose)

        laser2_data = self.laser3.getLaserData()
        laser2 = self.parse_laser_data(laser2_data)
        self.navigation.updateMap(laser2,2,pose)
        '''
    
    def parse_laser_data(self, laser_data):
        laser = []
        for i in range(laser_data.numLaser):
            dist = laser_data.distanceData[i]/1000.0
            angle = math.radians(i)
            laser.append([dist, angle])#
        return laser

#laser1~ 0 front      
#region floyd
