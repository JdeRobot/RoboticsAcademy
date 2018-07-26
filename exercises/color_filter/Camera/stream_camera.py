#
# Created on Jan, 2018
#
# @author: naxvm
#
# Class which abstracts a Camera from a proxy (created by ICE/ROS),
# and provides the methods to keep it constantly updated. Also, delivers it
# to the neural network, which returns returns the same image with the
# detected classes and scores, and the bounding boxes drawn on it.
#

import traceback
import threading
import numpy as np

class Camera:

    def __init__ (self, cam):
        ''' Camera class gets images from live video and transform them
        in order to detect objects in the image.
        '''

        self.cam = cam
        self.lock = threading.Lock()

        if self.cam.hasproxy():
            self.im = self.cam.getImage()
            self.height = self.im.height
            self.width = self.im.width

            print('Image size: {0}x{1} px'.format(
                    self.width, self.height))
        else:
            raise SystemExit("Interface camera not connected")

        self.trackImage = np.zeros((self.height, self.width,3), np.uint8)
        self.trackImage.shape = self.height, self.width, 3

        self.thresholdImage = np.zeros((self.height,self. width,1), np.uint8)
        self.thresholdImage.shape = self.height, self.width,


    def getImage(self):
        ''' Gets the image from the webcam and returns it. '''
        if self.cam:
            im = np.zeros((self.im_height, self.im_width, 3), np.uint8)
            im = np.frombuffer(self.im.data, dtype=np.uint8)
            im = np.reshape(im, (self.im_height, self.im_width, 3))

            return im

    def update(self):
        ''' Updates the camera with an incoming stream every time the thread changes. '''
        if self.cam:
            self.lock.acquire()

            self.im = self.cam.getImage()
            self.im_height = self.im.height
            self.im_width = self.im.width

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
