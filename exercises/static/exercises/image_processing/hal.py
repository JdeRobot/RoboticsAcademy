import cv2
import os
import logging

class HAL:

    def __init__(self):
        self.current_path = os.path.dirname(os.path.abspath(__file__))
        self.cameraCapture = cv2.VideoCapture(self.current_path + "/video.mp4")
        self.frame_number = 0

    def getImage(self, frame_number):
        self.cameraCapture.set(cv2.CAP_PROP_POS_FRAMES, frame_number)
        success, frame = self.cameraCapture.read()
        return frame
