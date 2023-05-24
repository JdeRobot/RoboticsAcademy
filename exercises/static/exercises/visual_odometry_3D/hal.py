import math
import threading
import time
from datetime import datetime
from glob import glob

import cv2
import numpy as np
from interfaces.camera import ListenerCamera, ListenerParameters
from interfaces.pinhole_camera import PinholeCamera

def read_kitti_groundtruth(groundtruth_file: str) -> list:
    with open(groundtruth_file, "r") as f:
        return f.readlines()
    

# Hardware Abstraction Layer
class HAL:
    def __init__(self):

        # Dataset data
        SEQUENCE_NUMBER = "01"
        DATASET = 'kitti'

        # Paths
        DATASET_PATH = "/datasets/" + DATASET + "/dataset/"
        SEQUENCE_PATH = DATASET_PATH + "sequences/" + SEQUENCE_NUMBER
        LEFT_CAMERA_IMAGES_PATH = SEQUENCE_PATH + "/image_0/*.png"
        RIGHT_CAMERA_IMAGES_PATH = SEQUENCE_PATH + "/image_1/*.png"
    
        # Files
        GROUNDTRUTH_FILE = DATASET_PATH + "poses/" + SEQUENCE_NUMBER + ".txt"
        CALIBRATION_FILE = SEQUENCE_PATH + "/calib.txt"

        # Images paths list
        self.image_counter = 0
        self.left_image_files_array = sorted(glob(LEFT_CAMERA_IMAGES_PATH))
        self.right_image_files_array = sorted(glob(RIGHT_CAMERA_IMAGES_PATH))

        # Grounth-truth variables
        self.groundtruth = read_kitti_groundtruth(GROUNDTRUTH_FILE)

        # Odometry variables
        self.image = None

    # User method
    # Get Image from ROS Driver Camera
    def get_image(self, lr):
        image = None
        if (lr == 'left'):
            image = cv2.imread(self.left_image_files_array[self.image_counter], 0)
        elif (lr == 'right'):
            image = cv2.imread(self.right_image_files_array[self.image_counter], 0)
        else:
            print("Invalid camera")
        return image

    # User method
    # Advance current frame
    def advance(self):
        self.image_counter += 1

    # User method
    # Get grounth-truth position in self.image_counter time
    def get_current_groundtruth(self):

        ss = self.groundtruth[self.image_counter].strip().split()
        x = float(ss[3])
        y = float(ss[7])
        z = float(ss[11])

        return np.array([x, y, z], dtype=np.float32).flatten().round(decimals=5)