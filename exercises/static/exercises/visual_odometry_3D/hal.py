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
    def __init__(self):

        # Dataset data
        SEQUENCE_NUMBER = "01"
        DATASET = 'kitti'

        # Paths
        DATASET_PATH = "/datasets/kitti/"
        SEQUENCE_PATH = DATASET_PATH + "sequences/" + SEQUENCE_NUMBER
        LEFT_CAMERA_IMAGES_PATH = SEQUENCE_PATH + "/image_0/*.png"
        RIGHT_CAMERA_IMAGES_PATH = SEQUENCE_PATH + "/image_1/*.png"
    
        # Files
        GROUNDTRUTH_FILE = DATASET_PATH + "poses/" + SEQUENCE_NUMBER + "/.txt"
        CALIBRATION_FILE = SEQUENCE_PATH + "/calib.txt"

        self.image = None

        self.image_counter = 0
        self.left_image_files_array = sorted(glob(LEFT_CAMERA_IMAGES_PATH))
        self.right_image_files_array = sorted(glob(RIGHT_CAMERA_IMAGES_PATH))

        print("BENITOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")

    # Get Image from ROS Driver Camera
    def getImage(self, lr):
        if (lr == 'left'):
            image = cv2.imread(self.left_image_files_array[self.image_counter], 0)
        elif (lr == 'right'):
            image = cv2.imread(self.right_image_files_array[self.image_counter], 0)
        else:
            print("Invalid camera")
        return image

    def advance(self):
        self.image_counter += 1
