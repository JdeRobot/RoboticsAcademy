import cv2

class HAL:

    def __init__(self):
        self.cameraCapture = cv2.VideoCapture(0)
        self.benchmark_vid_capture = cv2.VideoCapture("benchmarking/test_vid/video.avi")
        self.uploaded_vid_capture = cv2.VideoCapture("uploaded_video.mp4")
        self.frame_number = 0

    def getImage(self):
        success, frame = self.cameraCapture.read()
        return frame

    def getBenchmarkVid(self):
        try:
            detections = []
            self.benchmark_vid_capture.set(cv2.CAP_PROP_POS_FRAMES, self.frame_number)
            success, img = self.benchmark_vid_capture.read()
            file = open("benchmarking/groundtruths/" + str(self.frame_number) + ".txt", "r")
            for detection in file:
                detections.append(detection.strip().split())
            #self.frame_number += 1
            return success, img, detections, self.frame_number
        except:
            return False, [], [], 0

    def getVid(self):
        self.uploaded_vid_capture.set(cv2.CAP_PROP_POS_FRAMES, self.frame_number)
        success, img = self.uploaded_vid_capture.read()
        return success, img
        




