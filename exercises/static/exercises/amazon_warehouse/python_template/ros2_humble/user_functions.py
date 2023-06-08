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

    # Show image function
    def showImage(self, image):
        # Reshape to 3 channel if it has only 1 in order to display it
        if (len(image.shape) < 3):
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        self.shared_image.add(image)

    # Show Numpy function
    # def showNumpy(self, mat,h,w):
    #     self.shared_numpy = SharedNumpy("guiNumpy",h,w)
    #     self.shared_numpy.add(mat)