import json
import cv2
import base64
import numpy as np
import threading

from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from console_interfaces.general.console import start_console
from HAL import getPose3d

# Graphical User Interface Class

class GUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        # Payload vars
        self.payload = {'map': '', 'user': ''}
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)

        self.start()

    # Prepares and sends a map to the websocket server
    def update_gui(self):

        if np.any(self.left_image):
            _, encoded_left_image = cv2.imencode(".JPEG", self.left_image)
            b64_left = base64.b64encode(encoded_left_image).decode("utf-8")
            shape_left = self.left_image.shape
        else:
            b64_left = None
            shape_left = 0

        if np.any(self.right_image):
            _, encoded_right_image = cv2.imencode(".JPEG", self.right_image)
            b64_right = base64.b64encode(encoded_right_image).decode("utf-8")
            shape_right = self.right_image.shape
        else:
            b64_right = None
            shape_right = 0

        payload_left = {
            "image_left": b64_left,
            "shape_left": shape_left,
        }
        payload_right = {
            "image_right": b64_right,
            "shape_right": shape_right,
        }
        self.msg["image_left"] = json.dumps(payload_left)
        self.msg["image_right"] = json.dumps(payload_right)
        message = json.dumps(self.msg)
        self.send_to_client(message)

    # Functions to set the next image to be sent
    def setLeftImage(self, image):
        with self.image_lock:
            self.left_image = image

    def setRightImage(self, image):
        with self.image_lock:
            self.right_image = image

host = "ws://127.0.0.1:2303"
gui = GUI(host)

# Redirect the console
start_console()

# Expose the user functions
def showImage(image):
    gui.setRightImage(image)

def showLeftImage(image):
    gui.setLeftImage(image)