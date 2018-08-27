import threading
import time
from datetime import datetime
import cv2
import numpy as np
import math

from parallelIce.navDataClient import NavDataClient
from parallelIce.cmdvel import CMDVel
from parallelIce.extra import Extra
from parallelIce.pose3dClient import Pose3DClient


time_cycle = 80

hmin = 20
smin= 0
vmin= 0
hmax= 60
smax= 130
vmax= 130

class MyAlgorithm(threading.Thread):

    def __init__(self, camera, navdata, pose, cmdvel, extra):
        self.camera = camera
        self.navdata = navdata
        self.pose = pose
        self.cmdvel = cmdvel
        self.extra = extra

        self.minError=10
        self.prev_section=0

        self.height = 240
        self.width = 320

        self.image=None

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

    def detectRoad(self, img, row):

        # Returns an array with horizontal positions of road sections in a given row_number

        c = 0  # Counts section pixels
        d = 0  # Count no-section pixels
        sections = []
        section_found = False
        sect_start = sect_end = 0
        for i in range(0, self.width - 1):
            if img[row][i][0] > hmin and img[row][i][0] < hmax and img[row][i][1] > smin and img[row][i][
                1] < smax and img[row][i][2] > vmin and img[row][i][2] < vmax:
                d = 0
                if c == 0:  # Possible section found
                    sect_start = i
                c += 1
                if c < 7:  # Wait until 7 consecutive pixels
                    continue
                section_found = True

            else:  # Not a road pixel
                if section_found:
                    if d == 0:
                        sect_end = i
                    d += 1
                    if d < 7:  # Wait until 7 consecutive pixels
                        continue
                    else:
                        section_found = False
                        sect_center = (sect_start + sect_end) / 2
                        sections.append(sect_center)
                c = 0

        return sections

    def findCloserSection(self, sections):
        # Finds the closer road section within a row
        # Sections array must contain pixel positions (without changing coordinates)

        if len(sections) == 0:
            return None
        distance = abs(sections[0] - self.prev_section)
        ret_section = sections[0]
        for section in sections:
            if abs(section - self.prev_section) < distance:
                distance = abs(section - self.prev_section)
                ret_section = section
        return ret_section

    def execute(self):
        # Add your code here

        input_image = self.camera.getImage().data
        if input_image is not None:
            img = np.copy(input_image)

            image_HSV = cv2.cvtColor(input_image, cv2.COLOR_RGB2HSV)

            tmp = cv2.GaussianBlur(input_image, (11, 11), 0)
            tmp = cv2.cvtColor(tmp, cv2.COLOR_RGB2HSV)

            value_min_HSV = np.array([hmin, smin, vmin], dtype=np.uint8)
            value_max_HSV = np.array([hmax, smax, vmax], dtype=np.uint8)

            #Treshold image
            image_HSV_filtered = cv2.inRange(tmp, value_min_HSV, value_max_HSV)

            #image_HSV_filtered_Mask = np.dstack((image_HSV_filtered, image_HSV_filtered, image_HSV_filtered))

            _, contours, hierarchy = cv2.findContours(image_HSV_filtered, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE);
            maxArea = 1
            for contour in contours:
                epsilon = 0.1 * cv2.arcLength(contour, True);
                approx = cv2.approxPolyDP(contour, epsilon, True);
                x, y, w, h = cv2.boundingRect(contour);
                if (w * h) > maxArea:
                    self.xObject = x + (w / 2)
                    self.yObject = y + (h / 2)
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    maxArea = w * h

            row1 = self.height / 8
            row2 = row1 * 2
            row3 = row2 * 3
            rows = [row1, row2, row3]

            for row in rows:
                sections = self.detectRoad(img, row)
                if len(sections) == 0:
                    continue
                refx = self.findCloserSection(sections)
                refy = row
                break

            if len(sections) == 0:
                return

            # Change coordinates
            yImage = (self.height / 2) - refy
            xImage = refx - (self.width / 2)

            # Check position
            if abs(xImage) > self.minError:
                if xImage > 0:
                    self.cmdvel.setYaw(-0.2)
                else:
                    self.cmdvel.setYaw(0.2)
            else:
                self.cmdvel.setYaw(0)

            if yImage > self.minError:
                h = math.sqrt(yImage ** 2 + xImage ** 2)
                if h != 0:
                    self.cmdvel.setVX(0.5 * yImage / h)
                else:
                    self.cmdvel.setVX(0.5)
            else:
                self.cmdvel.setVX(0)

            self.cmdvel.sendVelocities()
            self.prev_section = refx

            self.setImageFiltered(img)
            '''
            If you want show a thresold image (black and white image)
            self.camera.setThresholdImage(bk_image)
            '''

