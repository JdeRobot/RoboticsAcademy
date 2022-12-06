import math
import threading
import time
from datetime import datetime
from glob import glob

import cv2
import numpy as np
from interfaces.camera import ListenerCamera, ListenerParameters
from interfaces.pinhole_camera import PinholeCamera


# Hardware Abstraction Layer
class HAL:
    # Dataset data
    SEQUENCE_NUMBER = "01"
    DATASET = 'kitti'

    # Paths
    DATASET_PATH = "/RoboticsAcademy/exercises/static/exercises/assets/kitti/dataset/"
    SEQUENCE_PATH = DATASET_PATH + "sequences/" + SEQUENCE_NUMBER
    LEFT_CAMERA_IMAGES_PATH = SEQUENCE_PATH + "/image_0/*.png"
    RIGHT_CAMERA_IMAGES_PATH = SEQUENCE_PATH + "/image_1/*.png"
   
    # Files
    GROUNDTRUTH_FILE = DATASET_PATH + "poses/" + SEQUENCE_NUMBER + "/.txt"
    CALIBRATION_FILE = SEQUENCE_PATH + "/calib.txt"

    # Class global variables
    IMAGE_COUNTER = 0
    LEFT_IMAGE_FILES_ARRAY = sorted(glob(LEFT_CAMERA_IMAGES_PATH))
    RIGHT_IMAGE_FILES_ARRAY = sorted(glob(RIGHT_CAMERA_IMAGES_PATH))

    def __init__(self):

        self.image = None
        self.counter = IMAGE_COUNTER


    # Get Image from ROS Driver Camera
    def getImage(self, lr):

        print("\t< LEFT_IMAGE_FILES_ARRAY >", LEFT_IMAGE_FILES_ARRAY)
        print("\t< RIGHT_IMAGE_FILES_ARRAY >", RIGHT_IMAGE_FILES_ARRAY)

        if (lr == 'left'):
            image = cv2.imread(LEFT_IMAGE_FILES_ARRAY[IMAGE_COUNTER], 0)
        elif (lr == 'right'):
            image = cv2.imread(RIGHT_IMAGE_FILES_ARRAY[IMAGE_COUNTER], 0)
        else:
            print("Invalid camera")
        
        self.counter += 1

        return image

    def advance(self):
        IMAGE_COUNTER += 1
