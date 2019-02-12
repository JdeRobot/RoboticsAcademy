import threading
import time
from datetime import datetime
import cv2
import numpy as np
import math

time_cycle = 80
value_min_HSV = np.array([20, 0, 0])
value_max_HSV = np.array([100, 130, 130])

class MyAlgorithm(threading.Thread):

    def __init__(self, drone):
        self.drone = drone

        self.height = 240
        self.width = 320

        self.yaw = 0.0

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
            image_HSV_V = cv2.cvtColor(input_imageV, cv2.COLOR_RGB2HSV)

            #Treshold image
            image_HSV_filtered_V = cv2.inRange(image_HSV_V, value_min_HSV, value_max_HSV)

            #Reducing noise
            opening_V = cv2.morphologyEx(image_HSV_filtered_V, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
            closing_V = cv2.morphologyEx(opening_V, cv2.MORPH_CLOSE, np.ones((10,10),np.uint8))

            #Filtered image
            image_HSV_filtered_Mask_V = np.dstack((closing_V, closing_V, closing_V))

            #drawing contours
            imgray_V = cv2.cvtColor(image_HSV_filtered_Mask_V, cv2.COLOR_BGR2GRAY)
            ret_V, thresh_V = cv2.threshold(imgray_V, 127, 255, 0)
            _, contours_V, hierarchy_V = cv2.findContours(thresh_V, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image_HSV_filtered_Mask_V, contours_V, -1, (0,255,0), 3)

            #Getting the centre of the road
            area = []
            for pic, contour in enumerate(contours_V):
                area.append(cv2.contourArea(contour))

            if len(area) > 1:
                if area[0] < area[1]:
                    M = cv2.moments(contours_V[1])
                else:
                    M = cv2.moments(contours_V[0])

            else:
                try:
                    M = cv2.moments(contours_V[0])
                except IndexError:
                    self.drone.sendCMDVelocities(0,0.3,0,0)
                    M = cv2.moments(0)

            if int(M['m00']) != 0:
                #print("Road detected")
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

            #move the drone
                if cy > 120:
                    self.drone.sendCMDVelocities(0,0.3,0,0.2)
                    print("Turning")
                elif cx < 20:
                    print("Detected two roads")
                    self.drone.sendCMDVelocities(0,0.3,0.1,0.0)
                else:
                    self.drone.sendCMDVelocities(0,0.3,0,0.0)

                print("cx: " + str(cx) + " cy: " + str(cy))
                self.yaw = int(cx)

                #drawing the center
                cv2.circle(image_HSV_filtered_Mask_V, (cx, cy), 7, np.array([255, 0, 0]), -1)


        if input_imageF is not None:
            image_HSV_F = cv2.cvtColor(input_imageF, cv2.COLOR_RGB2HSV)

            #Treshold image
            image_HSV_filtered_F = cv2.inRange(image_HSV_F, value_min_HSV, value_max_HSV)

            #Reducing noise
            opening_F = cv2.morphologyEx(image_HSV_filtered_F, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))


            image_HSV_filtered_Mask_F = np.dstack((opening_F, opening_F, opening_F))

            #drawing contours
            imgray_F = cv2.cvtColor(image_HSV_filtered_Mask_F, cv2.COLOR_BGR2GRAY)
            ret_F, thresh_F = cv2.threshold(imgray_F, 127, 255, 0)
            _, contours_F, hierarchy_F = cv2.findContours(thresh_F, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image_HSV_filtered_Mask_F, contours_F, -1, (0,255,0), 3)

            #printing the filtered image
            self.setImageFilteredVentral(image_HSV_filtered_Mask_V)
            self.setImageFilteredFrontal(image_HSV_filtered_Mask_F)
