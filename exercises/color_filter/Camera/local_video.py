#
# Created on Jan, 2018
#
# @author: naxvm
#
# Class which abstracts a Camera from a local video file,
# and provides the methods to keep it constantly updated. Also, delivers it
# to the neural network, which returns returns the same image with the
# detected classes and scores, and the bounding boxes drawn on it.
#

import traceback
import threading
import cv2
from os import path
import numpy as np

class Camera:

    def __init__ (self, video_path):
        ''' Camera class gets images from a video device and transform them
        in order to detect objects in the image.
        '''
        self.lock = threading.Lock()
        video_path = path.expanduser(video_path)

        if not path.isfile(video_path):
            raise SystemExit('%s does not exists. Please check the path.' % (video_path))

        self.cam = cv2.VideoCapture(video_path)
        if not self.cam.isOpened():
            print("%d is not a valid device index in this machine." % (device_idx))
            raise SystemExit("Please check your camera id (hint: ls /dev)")

        self.width = self.cam.get(3)
        self.height = self.cam.get(4)
        self.image = np.zeros((self.height, self.width,3), np.uint8)

        self.trackImage = np.zeros((self.height, self.width,3), np.uint8)
        self.trackImage.shape = self.height, self.width, 3

        self.thresholdImage = np.zeros((self.height,self. width,1), np.uint8)
        self.thresholdImage.shape = self.height, self.width,


    def getImage(self):
        ''' Gets the image from the webcam and returns it. '''
        return self.image

    def update(self):
        ''' Updates the camera with a frame from the device every time the thread changes. '''
        if self.cam:
            self.lock.acquire()
            _, frame = self.cam.read()
            self.image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.lock.release()

    def getColorImage(self):
        img = np.zeros((self.height, self.width,3), np.uint8)
        img = self.trackImage
        img.shape = self.trackImage.shape
        return img

    def setColorImage(self,image):
        self.trackImage = image
        self.trackImage.shape = image.shape

    def getThresholdImage(self):
        img = np.zeros((self.height, self.width,1), np.uint8)
        img = self.thresholdImage
        img.shape = self.thresholdImage.shape
        return img

    def setThresholdImage(self,image):
        self.thresholdImage = image
        self.thresholdImage.shape = image.shape
