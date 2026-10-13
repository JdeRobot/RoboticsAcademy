import json
import cv2
import base64
import threading
import time
import numpy as np

from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console

# Graphical User Interface Class


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):
        super().__init__(host)

        # Execution control vars
        self.right_image = None
        self.image_lock = threading.Lock()
        self.msg = {"image_right": ""}

        self.start()

    # Process outcoming messages from the GUI
    def gui_out_thread(self):
        while self.running:
            start_time = time.time()
            self.iteration_counter += 1

            # Check if a new image should be sent
            with self.ack_lock:
                with self.image_lock:
                    if self.ack:
                        if np.any(self.right_image):
                            self.update_gui()
                            self.ack = False

            # Maintain desired frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, self.out_period - elapsed)
            time.sleep(sleep_time)

    # Prepares and send image to the websocket server
    def update_gui(self):

        if np.any(self.right_image):
            _, encoded_right_image = cv2.imencode(".JPEG", self.right_image)
            b64_right = base64.b64encode(encoded_right_image).decode("utf-8")
            shape_right = self.right_image.shape
        else:
            b64_right = None
            shape_right = 0

        payload_right = {
            "image_right": b64_right,
            "shape_right": shape_right,
        }

        self.msg["image_right"] = json.dumps(payload_right)
        message = json.dumps(self.msg)
        self.send_to_client(message)

    # Functions to set the next image to be sent
    def setRightImage(self, image):
        with self.image_lock:
            self.right_image = image


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

# Redirect the console
start_console()


# Expose the user functions
def showImage(image):
    gui.setRightImage(image)
