import json
import subprocess
import cv2
import base64
import threading
import time
import numpy as np

from map import Map

from gui_interfaces.general.measuring_threading_gui_harmonic import MeasuringThreadingGUI
from console_interfaces.general.console import start_console

from HAL import getPose3d, getOdom

class GUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):
        super().__init__(host)

        self.image = None
        self.image_lock = threading.Lock()

        self.predict_pose = None
        self.map = Map(getPose3d, getOdom)
        
        # Payload vars
        self.payload = {"image": "", "real_pose": "","noisy_pose": "", "estimate_pose": "" }

        self.start()

    # Prepares and send image to the websocket server
    def update_gui(self):
        pos_message = self.map.getRobotCoordinates()
        pos_message = str(pos_message)
        self.payload["real_pose"] = pos_message

        n_pos_message = self.map.getRobotCoordinatesWithNoise()
        n_pos_message = str(n_pos_message)
        self.payload["noisy_pose"] = n_pos_message

        if np.any(self.predict_pose):
            p_pos_message = str(self.predict_pose)
            self.payload["estimate_pose"] = p_pos_message

        if np.any(self.image):
            _, encoded_image = cv2.imencode(".JPEG", self.image)
            b64 = base64.b64encode(encoded_image).decode("utf-8")
            shape = self.image.shape
        else:
            b64 = None
            shape = 0

        payload_img = {
            "image": b64,
            "shape": shape,
        }

        self.payload["image"] = json.dumps(payload_img)
        message = json.dumps(self.payload)
        self.send_to_client(message)

    # Functions to set the next image to be sent
    def setImage(self, image):
        with self.image_lock:
            self.image = image
    
    def setEstimatedRobotPose(self, pose):
        self.predict_pose = pose


host = "ws://127.0.0.1:2303"
gui = GUI(host)

# Redirect the console
start_console()

# Expose the user functions
def showImage(image):
    gui.setImage(image)

def showEstimatedPose(pose):
    """Pose must be (x, y, yaw)"""
    x, y, yaw = pose

    scale_y = -85
    offset_y = -6.88
    y = scale_y * (offset_y - y)

    scale_x =  83
    offset_x = 8
    x = scale_x * (offset_x - x)

    transformed_pose = (x, y, yaw)
    
    gui.setEstimatedRobotPose(transformed_pose)

def followRobot():
    # Execute Later: gz topic -t /gui/track -m gz.msgs.CameraTrack -p 'track_mode:2' to trigger the camera tracker
    for i in range(10):
        subprocess.call(
            "gz topic -t /gui/track -m gz.msgs.CameraTrack -p 'track_mode:2'",
            shell=True,
            stderr=subprocess.STDOUT,
            bufsize=1024,
            universal_newlines=True,
        )

# TODO: if we add the ability to unfollow, then we need to put the robot name here
# def unfollowRobot():
#     # Execute Later: gz topic -t /gui/track -m gz.msgs.CameraTrack -p 'track_mode:2' to trigger the camera tracker
#     for i in range(10):
#         subprocess.call(
#             "gz topic -t /gui/track -m gz.msgs.CameraTrack -p 'track_mode:0'",
#             shell=True,
#             stderr=subprocess.STDOUT,
#             bufsize=1024,
#             universal_newlines=True,
#         )