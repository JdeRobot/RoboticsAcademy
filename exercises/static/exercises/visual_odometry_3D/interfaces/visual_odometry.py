import cv2
import pandas as pd
import numpy as np

from .maths.cv_maths import feature_tracking
from .maths import rotation2Euler
from ...visual_odometry_3D.interfaces.pinhole_camera import PinholeCamera
from .scales import (get_absolute_scale_euromav, get_absolute_scale_kitti,
                     get_absolute_scale_vkitti2)
from .scales import (get_true_rotation_kitti)

STAGE_FIRST_FRAME = 0
STAGE_SECOND_FRAME = 1
STAGE_DEFAULT_FRAME = 2
kMinNumFeature = 1500

def read_groundtruth(groundtruth_file: str, dataset: str) -> list:

    if dataset == "vkitti2":
        df = pd.read_csv(groundtruth_file, sep=" ")
        return df[df["cameraID"] == 0].copy()

    with open(groundtruth_file, "r") as f:
        return f.readlines()


class VisualOdometry:

    def __init__(self, camera: PinholeCamera, groundtruth_file: str, dataset: str):
        
        self.dataset = dataset
        self.camera = camera
        
        self.frame_stage = 0
        
        self.new_frame = None
        self.last_frame = None
        
        self.cur_R = np.eye(3)
        self.cur_t = None
        self.px_ref = None
        self.px_cur = None
        self.focal = camera.fu
        self.pp = (camera.cu, camera.cv)
        
        self.trueX, self.trueY, self.trueZ = 0, 0, 0
        self.true_R = np.zeros( shape=(3,3) )
        self.true_t = np.zeros((1,3))
        
        self.detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
        self.groundtruth = read_groundtruth(groundtruth_file, dataset)
        
        self.timestamp = None
        self.frame_timestamps_list = None
        self.timestamp_groundtruth_list = None


    def calculate_true_rotration(self, frame_id:int) -> None:

        if self.dataset == "kitti":
            self.true_R = get_true_rotation_kitti(self.groundtruth, frame_id)

        if self.dataset == "vkitti2":
            pass

        if self.dataset == "eurocmav":
            pass  

    def calculate_absolute_scale(self, frame_id: int) -> float:

        if self.dataset == "kitti":
            scale, truth = get_absolute_scale_kitti(self.groundtruth, frame_id)
            self.true_t = truth
            return scale
        
        if self.dataset == "vkitti2":
            scale, truth = get_absolute_scale_vkitti2(self.groundtruth, frame_id)
            self.true_t = truth
            return scale

        if self.dataset == "eurocmav":
            scale, truth = get_absolute_scale_euromav(self.groundtruth, self.timestamp_groundtruth_list , self.frame_timestamps_list, frame_id)
            self.true_t = truth
            return scale

        print("Dataset not supported.")
        exit(1)


    def process_first_frame(self) -> None:

        self.px_ref = self.detector.detect(self.new_frame)
        self.px_ref = np.array([x.pt for x in self.px_ref], dtype=np.float32)
        _ = self.calculate_absolute_scale(0)
        self.calculate_true_rotration(0)
        self.frame_stage = STAGE_SECOND_FRAME
    

    def process_second_frame(self) -> None:

        self.px_ref, self.px_cur = feature_tracking(self.last_frame, self.new_frame, self.px_ref)
        E, _ = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        _, self.cur_R, self.cur_t, _ = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)
        self.px_ref = self.px_cur
        _ = self.calculate_absolute_scale(1)
        self.calculate_true_rotration(1)
        self.frame_stage = STAGE_DEFAULT_FRAME


    def process_frame(self, frame_id: int) -> None:

        self.px_ref, self.px_cur = feature_tracking(self.last_frame, self.new_frame, self.px_ref)
        E, _ = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        _, R, t, _ = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)
        absolute_scale = self.calculate_absolute_scale(frame_id)
        self.calculate_true_rotration(frame_id)
        
        if(absolute_scale > 0.1):
            self.cur_t = self.cur_t + absolute_scale * self.cur_R @ t 
            self.cur_R = R @ self.cur_R

        if(self.px_ref.shape[0] < kMinNumFeature):
            self.px_cur = self.detector.detect(self.new_frame)
            self.px_cur = np.array([x.pt for x in self.px_cur], dtype=np.float32)
            
        self.px_ref = self.px_cur


    def update(self, img: np.ndarray, frame_id: int, R=None, t=None) -> None:
        assert (
            img.ndim == 2
            and img.shape[0] == self.camera.height
            and img.shape[1] == self.camera.width
        ), "Frame: provided image has not the same size as the camera model or image is not grayscale"

        self.new_frame = img

        if self.frame_stage == STAGE_DEFAULT_FRAME:
            self.process_frame(frame_id)

        elif self.frame_stage == STAGE_SECOND_FRAME:
            self.process_second_frame()

        elif self.frame_stage == STAGE_FIRST_FRAME:
            self.process_first_frame()

        self.last_frame = self.new_frame

    def get_true_euler_angles(self):
        return rotation2Euler(self.true_R)

    def get_estimated_euler_angles(self):
        return rotation2Euler(self.cur_R)