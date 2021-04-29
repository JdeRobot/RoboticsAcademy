import threading
import time
import math
import rosbag
import cv2
import numpy as np
from datetime import datetime


time_cycle = 40 #80

class MyAlgorithm(threading.Thread):

    def __init__(self, bag_readings, pose_obj):
        self.bag_readings = bag_readings
        self.pose_obj = pose_obj
        self.threshold_image = np.zeros((640,480,3), np.uint8)
        self.color_image = np.zeros((640,480,3), np.uint8)
        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        self.threshold_image_lock = threading.Lock()
        self.color_image_lock = threading.Lock()
        self.pose_lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)
        self.diff_time = 0

        
    def getReadings(self , *sensors):
        self.lock.acquire()
        data = self.bag_readings.getData(sensors)
        self.lock.release()
        return data

    def set_predicted_path(self,path):
        self.pose_lock.acquire()
        
        self.pose_obj.set_pred_path(path)
        self.pose_lock.release()

    def set_predicted_pose(self,x,y,t):
        self.pose_lock.acquire()
        self.predicted_pose = [x,y]
        self.pose_obj.set_pred_pose([x,y],t)
        self.pose_lock.release()

    def get_predicted_pose(self):
        self.pose_lock.acquire()
        
    def set_processed_image(self,image):
        img = np.copy(image)
        if len(img.shape) == 2:
          img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        self.threshold_image_lock.acquire()
        self.threshold_image = img
        self.threshold_image_lock.release()
    def get_processed_image (self):
        self.threshold_image_lock.acquire()
        img  = np.copy(self.threshold_image)
        self.threshold_image_lock.release()
        return img

    def run (self):

        #self.algo_start_time = time.time()
        while (not self.kill_event.is_set()):
            start_time = datetime.now()

            if not self.stop_event.is_set():
                self.algo_start_time = time.time()
                self.algorithm()
                self.algo_stop_time = time.time()
                self.diff_time = self.diff_time + (self.algo_stop_time - self.algo_start_time)
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

    def algorithm(self):
        #Getting readings data
        data = self.getReadings('accelerometer' , 'orientation' , 'color_img' , 'depth_img') # to get readings data from particular sensors
        
        
        
        #data = self.getReadings('stream') # to stream data from all sensors one by one 
    
        
        #imu data
        ax=data.accelerometer['x']
        ay=data.accelerometer['y']
        accelerometer_t = data.accelerometer_t
        orientation_t = data.orientation_t
        qz=data.orientation['qz']
        qw=data.orientation['qw']
        #color image
        color_image = data.color_img
        #depth image
        depth_image = data.depth_img
        color_img_t = data.color_img_t
        
        
        
      
        
        x = 1 
        y = 1
        #Show processed image on GUI
        self.set_processed_image(color_image)
        #self.set_processed_image(depth_image)
        #set predicted pose
        self.set_predicted_pose(x,y,color_img_t)

        #set predicted path at once /or reset the previously set predicted poses at once ---- path should be Nx2 numpy array or python list [x,y].
        #self.set_predicted_path(path)