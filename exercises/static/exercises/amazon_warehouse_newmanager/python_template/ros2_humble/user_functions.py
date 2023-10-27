from shared.image import SharedImage
from shared.value import SharedValue
from shared.numpy import SharedNumpy
import numpy as np
import cv2

# Define GUI functions
class GUIFunctions:
    def __init__(self):
        # Initialize image variable
        self.shared_image = SharedImage("guiimage")

    def showPath(self, image):
        print("Show Path Function")
