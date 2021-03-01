
import cv2
import numpy as np


class ReceiveImage():

    def __init__(self):
        self.cameraCapture = cv2.VideoCapture(0)
        self.fps = 30  # An assumption

    def getImage(self):
        success, frame = self.cameraCapture.read()
        return frame


if __name__ == '__main__':
    pass