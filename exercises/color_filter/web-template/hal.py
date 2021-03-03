import cv2


class HAL:

    def __init__(self):
        self.cameraCapture = cv2.VideoCapture(0)

    def getImage(self):
        success, frame = self.cameraCapture.read()
        return frame


if __name__=="__main__":

    prueba = HAL()
    print(prueba.getImage())