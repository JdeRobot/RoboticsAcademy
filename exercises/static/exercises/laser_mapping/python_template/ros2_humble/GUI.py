import json
import cv2
import base64
import threading
import time
import numpy as np
import math

from map import Map

from gui_interfaces.general.measuring_threading_gui_harmonic import MeasuringThreadingGUI
from console_interfaces.general.console import start_console

from HAL import getPose3d, getOdom

class GUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):
        super().__init__(host)

        self.user_map = None
        self.image_lock = threading.Lock()

        self.map = Map(getPose3d, getOdom)
        
        # Payload vars
        self.payload = {"user_map": "", "real_pose": "","noisy_pose": ""}

        self.start()

    # Prepares and send image to the websocket server
    def update_gui(self):
        pos_message = self.map.getRobotCoordinates()
        pos_message = str(pos_message)
        self.payload["real_pose"] = pos_message

        n_pos_message = self.map.getRobotCoordinatesWithNoise()
        n_pos_message = str(n_pos_message)
        self.payload["noisy_pose"] = n_pos_message

        if np.any(self.user_map):
            _, encoded_image = cv2.imencode(".JPEG", self.user_map)
            b64 = base64.b64encode(encoded_image).decode("utf-8")
            shape = self.user_map.shape
        else:
            b64 = None
            shape = 0

        payload_img = {
            "user_map": b64,
            "shape": shape,
        }

        self.payload["user_map"] = json.dumps(payload_img)
        message = json.dumps(self.payload)
        self.send_to_client(message)

    # Function to set the next image to be sent
    def setUserMap(self, image):
        if image.shape[0] != 970 or image.shape[1] != 1500:
            raise ValueError('map passed has the wrong dimensions, it has to be 970 pixels high and 1500 pixels wide')
        processed_image = np.stack((image,) * 3, axis=-1)
        with self.image_lock:
            self.user_map = processed_image
    
    def poseToMap(self, x_prime, y_prime, yaw_prime):
        y = -23.58 * ( -20.36 - x_prime)
        x = -23.53  * ( -31.95 - y_prime)
        yaw = yaw_prime - math.pi/2
        return [round(x), round(y), yaw]


host = "ws://127.0.0.1:2303"
gui = GUI(host)

# Redirect the console
start_console()

# Expose the user functions
def setUserMap(image):
    gui.setUserMap(image)

def poseToMap(x_prime, y_prime, yaw_prime):
    return gui.poseToMap(x_prime, y_prime, yaw_prime)
