import json
import cv2
import base64
import threading
import time
import numpy as np
from gui_interfaces.general.measuring_threading_gui_harmonic import MeasuringThreadingGUI
from console_interfaces.general.console import start_console


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.payload = {"image": ""}

        self.start()

    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image_to_be_shown = self.image_to_be_shown

        payload = {"image": "", "shape": ""}

        if not image_to_be_shown_updated or image_to_be_shown is None:
            return payload

        shape = image_to_be_shown.shape
        frame = cv2.imencode(".JPEG", image_to_be_shown)[1]
        encoded_image = base64.b64encode(frame)

        payload["image"] = encoded_image.decode("utf-8")
        payload["shape"] = shape

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload

    def update_gui(self):
        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def setImage(self, image):
        """Single point of entry for all images"""
        with self.image_show_lock:
            self.image_to_be_shown = image
            self.image_to_be_shown_updated = True


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)
start_console()


def showImage(image):
    gui.setImage(image)
