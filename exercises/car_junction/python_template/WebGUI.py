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


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):

        self.out_period = 1.0 / freq
        self.image = None
        self.image_lock = threading.Lock()
        self.ack = True
        self.ack_frontend = True
        self.ack_lock = threading.Lock()
        self.running = True

        self.world_name = "empty"

        self.host = host
        self.msg = {"image": ""}

        self.ideal_cycle = 80
        self.real_time_factor = 0
        self.frequency_message = {"brain": "", "gui": "", "rtf": ""}
        self.iteration_counter = 0
        self.fps = 0
        self.lat = 0

        self.start()

    def gui_out_thread(self):
        while self.running:
            start_time = time.time()
            self.iteration_counter += 1

            # Check if a new image should be sent
            with self.ack_lock:
                with self.image_lock:
                    if self.ack:
                        if np.any(self.image):
                            self.update_gui()
                            self.ack = False

            # Maintain desired frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, self.out_period - elapsed)
            time.sleep(sleep_time)

    def update_gui(self):

        if np.any(self.image):
            _, encoded_front_image = cv2.imencode(".JPEG", self.image)
            b64_front = base64.b64encode(encoded_front_image).decode("utf-8")
        else:
            b64_front = None

        payload_front = {
            "image": b64_front,
        }
        self.msg["image"] = json.dumps(payload_front)
        message = json.dumps(self.msg)
        self.send_to_client(message)

    def setImage(self, image):
        with self.image_lock:
            self.image = image


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)
start_console()


def showImage(image):
    gui.setImage(image)
