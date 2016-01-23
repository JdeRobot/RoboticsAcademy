from sensors import sensor
import cv2
import numpy as np

class MyAlgorithm():

    def __init__(self, sensor):
        self.sensor = sensor

    def execute(self):
        # Add your code here

        input_image = self.sensor.getImage()
        if input_image != None:
            self.sensor.setColorImage(input_image)
            '''
            If you want show a thresold image (black and white image)
            self.sensor.setThresoldImage(bk_image)
            '''