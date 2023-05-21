import cv2


class HAL:

    def __init__(self):
        self.cameraCapture = cv2.VideoCapture(0)
        if self.cameraCapture.isOpened():
            print("Camera is Open")

    def getImage(self):
        success, frame = self.cameraCapture.read()
        return frame
