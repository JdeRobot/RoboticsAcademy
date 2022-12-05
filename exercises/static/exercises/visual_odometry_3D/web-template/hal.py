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
    DATASET_PATH = "exercises/static/exercises/assets/kitti/dataset/"
    SEQUENCE_PATH = DATASET_PATH + "sequences/" + SEQUENCE_NUMBER
    LEFT_CAMERA_IMAGES_PATH = SEQUENCE_PATH + "/image_0/*.png"
    RIGHT_CAMERA_IMAGES_PATH = SEQUENCE_PATH + "/image_1/*.png"
   
    # Files
    GROUNDTRUTH_FILE = DATASET_PATH + "poses/" + SEQUENCE_NUMBER + "/.txt"
    CALIBRATION_FILE = SEQUENCE_PATH + "/calib.txt"

    # Class variables
    IMAGE_COUNTER = 0
    LEFT_IMAGE_FILES_ARRAY = sorted(glob(LEFT_CAMERA_IMAGES_PATH))
    RIGHT_IMAGE_FILES_ARRAY = sorted(glob(RIGHT_CAMERA_IMAGES_PATH))

    def __init__(self):

        self.image = None


    # Get Image from ROS Driver Camera
    def get_image(self, lr):
        if (lr == 'left'):
            image = cv2.imread(LEFT_IMAGE_FILES_ARRAY[IMAGE_COUNTER], 0)
        elif (lr == 'right'):
            image = cv2.imread(RIGHT_IMAGE_FILES_ARRAY[IMAGE_COUNTER], 0)
        else:
            print("Invalid camera")

        return image

    def advance(self):
        IMAGE_COUNTER += 1
