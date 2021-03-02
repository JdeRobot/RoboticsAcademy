import cv2

class ConsumerImage():

    def __init__(self):
        self.cameraCapture = cv2.VideoCapture(0)
        self.fps = 30  # An assumption

    def getImage(self):
        success, frame = self.cameraCapture.read()
        return frame

