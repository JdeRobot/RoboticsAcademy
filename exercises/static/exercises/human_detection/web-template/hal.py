import cv2
import os


class HAL:

    def __init__(self):
        # Saving the current path for later use.
        # The current path is somehow required everytime for accessing files, when the exercise is running in the docker container.
        self.current_path = os.path.dirname(os.path.abspath(__file__))
        self.cameraCapture = cv2.VideoCapture(0)
        self.benchmark_vid_capture = cv2.VideoCapture(self.current_path + "/benchmarking/test_vid/video.avi")
        self.uploaded_vid_capture = cv2.VideoCapture(self.current_path + "/uploaded_video.mp4")
        #path to the ground truth detections directory
        self.gt_path = os.path.join(self.current_path, "benchmarking/groundtruths/")
        self.frame_number = 0

    def getImage(self):
        success, frame = self.cameraCapture.read()
        return frame

    def getBenchmarkVid(self):
        try:
            detections = []
            # After the frame has been inferred on, the frame number gets incremented in exercise.py
            self.benchmark_vid_capture.set(cv2.CAP_PROP_POS_FRAMES, self.frame_number)
            success, img = self.benchmark_vid_capture.read()
            # opening the ground_truth detection file for the corresponding frame number
            file = open(self.gt_path + str(self.frame_number) + ".txt", 'r')
            for detection in file:
                detections.append(detection.strip().split())
            return success, img, detections, self.frame_number
        except:
            return False, [], [], 0

    def getVid(self):
        # After the frame has been inferred on, the frame number gets incremented in exercise.py
        self.uploaded_vid_capture.set(cv2.CAP_PROP_POS_FRAMES, self.frame_number)
        success, img = self.uploaded_vid_capture.read()
        return success, img
        




