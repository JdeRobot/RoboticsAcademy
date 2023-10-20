import json
import math
import threading
import time
from datetime import datetime
from glob import glob

import cv2
import numpy as np
from interfaces.camera import ListenerCamera, ListenerParameters
from interfaces.pinhole_camera import PinholeCamera

ROTATION_CAMERA_TO_WORLD = np.array([
    [0, 0, 1],
    [0, 1, 0],
    [1, 0, 0]
])

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def rotation2Euler(R):
    """
    Calculates euler angles from rotation matrix.
    From horizon to body axes using Tait-Bryan angles.
    """

    assert (isRotationMatrix(R), "Range of matrix is not 3.")

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return -np.array([x, y, z])  # minus because of the rotation matrix convention

def read_kitti_groundtruth(groundtruth_file: str) -> list:
    with open(groundtruth_file, "r") as f:
        return f.readlines()
    

# Hardware Abstraction Layer
class HAL:
    def __init__(self):

        # Dataset data
        SEQUENCE_NUMBER = "00"
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
        
        self.init_true_position = self.get_groundtruth(0)
        self.init_true_rotation_matrix = self.get_true_rotation_matrix(0)
        
        self.true_position_corrected = np.zeros((1,3))
        self.true_rotation_matrix_corrected = np.zeros( shape=(3,3) )

        # Odometry variables
        self.image = None
        self.estimated_position = np.array([0, 0, 0], dtype=np.float32).flatten().round(decimals=5)
        self.estimated_euler_angles = np.array([0, 0, 0], dtype=np.float32).flatten().round(decimals=5)

        # camera
        self.camera = PinholeCamera.from_kitti(file_path=CALIBRATION_FILE, width=1241, height=376)

    # User method
    # Advance current frame
    def advance(self):
        self.image_counter += 1

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

    # Get grounth-truth position in frame_id time
    def get_groundtruth(self, frame_id: int):

        ss = self.groundtruth[frame_id].strip().split()
        x = float(ss[3])
        y = float(ss[7])
        z = float(ss[11])

        return np.array([x, y, z], dtype=np.float32).flatten().round(decimals=5)

    # User method
    # Get corrected grounth-truth position in self.image_counter time
    def get_current_groundtruth_position(self):
        self.true_position_corrected = self.get_groundtruth(self.image_counter) - self.init_true_position
        return self.true_position_corrected

    # Get true rotation matrix in frame_id time
    def get_true_rotation_matrix(self, frame_id: int) -> np.ndarray:

        ss = self.groundtruth[frame_id].strip().split()

        r11 = float(ss[0])
        r12 = float(ss[1])
        r13 = float(ss[2])

        r21 = float(ss[4])
        r22 = float(ss[5])
        r23 = float(ss[6])

        r31 = float(ss[8])
        r32 = float(ss[9])
        r33 = float(ss[10])

        R = np.array(
            [
                [r11, r12, r13],
                [r21, r22, r23],
                [r31, r32, r33],
            ]
            , dtype=np.float32)

        R = np.round(R, decimals=7)

        return R

    def get_true_rotation_matrix_corrected(self):
        self.true_rotation_matrix_corrected = np.linalg.inv(self.init_true_rotation_matrix) @ self.get_true_rotation_matrix(self.image_counter)
        return self.true_rotation_matrix_corrected

    def get_true_euler_angles_corrected_array(self):
        roll, pitch, yaw = rotation2Euler( self.get_true_rotation_matrix_corrected() )
        return np.array([roll, pitch, yaw], dtype=np.float32).flatten().round(decimals=5)
    
    def get_true_euler_angles_corrected(self):
        true_euler_angles_corrected = rotation2Euler( self.get_true_rotation_matrix_corrected() )
        trueRoll, truePitch, trueYaw = true_euler_angles_corrected
        message = {
            "yaw":   str(trueYaw),
            "pitch": str(truePitch),
            "roll":  str(trueRoll)
            }
        return json.dumps(message)
    
    def get_estimated_euler_angles(self):
        estimatedRoll, estimatedPitch, estimatedYaw = self.estimated_euler_angles
        message = {
            "yaw":   str(estimatedYaw),
            "pitch": str(estimatedPitch),
            "roll":  str(estimatedRoll)
            }
        return json.dumps(message)
    
    def get_estimated_position(self):
        x, y, z = ROTATION_CAMERA_TO_WORLD @ self.estimated_position
        message = {
            "x": str(x),
            "y": str(y),
            "z": str(z)
            }
        return json.dumps(message)
    
    def get_true_position_corrected(self):
        x, y, z = ROTATION_CAMERA_TO_WORLD @ self.get_current_groundtruth_position()
        message = {
            "x": str(x),
            "y": str(y),
            "z": str(z)
            }
        return json.dumps(message)

    # User method
    # Set estimated position calculated by user
    def set_estimated_position(self, x: float, y: float, z: float):
        self.estimated_position = np.array([x, y, z], dtype=np.float32).flatten().round(decimals=5)
    
    # User method
    # Set estimated orientation in euler angles calculated by user
    def set_estimated_euler_angles(self, roll: float, pitch: float, yaw: float):
        self.estimated_euler_angles = np.array([roll, pitch, yaw], dtype=np.float32).flatten().round(decimals=5)

    # User method
    # Return the pinhole camera model
    def get_camera_model(self):
        return self.camera
    