#
#  Copyright (C) 1997-2016 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Alberto Martin Florido <almartinflorido@gmail.com>
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#
import numpy as np
import cv2
import threading
from parallelIce.cameraClient import CameraClient

class CameraSegment:

    def __init__(self, camera):
        self.lock = threading.Lock()
        self.client = camera

        img = self.client.getImage()

        self.height = 640
        self.width = 360


        if self.client.hasproxy():
            self.trackImage = np.zeros((self.height, self.width,3), np.uint8)
            self.trackImage.shape = self.height, self.width, 3

            self.detectImage = np.zeros((self.height, self.width,3), np.uint8)
            self.detectImage.shape = self.height, self.width, 3

            self.thresholdImage = np.zeros((self.height,self. width,3), np.uint8)
            self.thresholdImage.shape = self.height, self.width, 3
    
    def getImage(self):
        self.lock.acquire()
        img = self.client.getImage().data
        img = cv2.resize(img, (640, 360))
        self.lock.release()
        return img

    def getColorImage(self):
        if self.client.hasproxy():
            self.lock.acquire()
            img = np.zeros((self.height, self.width,3), np.uint8)
            img = self.trackImage
            img.shape = self.trackImage.shape
            self.lock.release()
            return img
        return None

    def setColorImage(self,image):
        if self.client.hasproxy():
            self.lock.acquire()
            self.trackImage = image
            self.trackImage.shape = image.shape
            self.lock.release()

    def getDetectImage(self):
        if self.client.hasproxy():
            self.lock.acquire()
            img = np.zeros((self.height, self.width,3), np.uint8)
            img = self.detectImage
            img.shape = self.detectImage.shape
            self.lock.release()
            return img
        return None

    def setDetectImage(self,image):
        if self.client.hasproxy():
            self.lock.acquire()
            self.detectImage = image
            self.detectImage.shape = image.shape
            self.lock.release()

    def getThresholdImage(self):
        if self.client.hasproxy():
            self.lock.acquire()
            img = np.zeros((self.height, self.width,3), np.uint8)
            img = self.thresholdImage
            img.shape = self.thresholdImage.shape
            self.lock.release()
            return img
        return None

    def setThresholdImage(self,image):
        if self.client.hasproxy():
            self.lock.acquire()
            self.thresholdImage = image
            self.thresholdImage.shape = image.shape
            self.lock.release()
