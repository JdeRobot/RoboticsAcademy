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
        super().__init__(host, freq, world_name="power_tower_inspection")

        # Execution control vars
        self.out_period = 1.0 / freq
        self.right_image = None
        self.left_image = None
        self.image_lock = threading.Lock()
        self.running = True

        self.msg = {"image_right": "", "image_left": ""}

        self.fps = 0
        self.lat = 0

        # Import HAL for reset functionality
        try:
            import HAL
            self.hal_module = HAL
        except ImportError:
            self.hal_module = None

        self.start()

    # Override to handle additional messages
    def gui_in_thread(self, ws, message):
        # Handle ack and start messages from parent class
        if "ack" in message:
            with self.ack_lock:
                self.ack = True
        elif "start" in message:
            with self.ack_lock:
                self.ack_frontend = True
        elif "pause" in message:
            # Handle pause command
            if self.hal_module and hasattr(self.hal_module, 'pause'):
                self.hal_module.pause()
                print("WebGUI: Drone paused", flush=True)
        elif "resume" in message:
            # Handle resume command
            if self.hal_module and hasattr(self.hal_module, 'resume'):
                self.hal_module.resume()
                print("WebGUI: Drone resumed", flush=True)
        elif "reset" in message:
            # Handle reset command
            if self.hal_module and hasattr(self.hal_module, 'reset'):
                self.hal_module.reset()
                print("WebGUI: Drone reset", flush=True)
        # Clear images on reset
        elif message == "clear":
            with self.image_lock:
                self.left_image = None
                self.right_image = None

    # Process outcoming messages from the GUI
    def gui_out_thread(self):
        while self.running:
            start_time = time.time()
            self.iteration_counter += 1

            # Check if a new image should be sent
            with self.ack_lock:
                with self.image_lock:
                    if self.ack:
                        if np.any(self.left_image) or np.any(self.right_image):
                            self.update_gui()
                            self.ack = False

            # Maintain desired frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, self.out_period - elapsed)
            time.sleep(sleep_time)

    # Prepares and send image to the websocket server
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

    def reset_gui(self):
        """Reset the GUI to initial state."""
        with self.image_lock:
            self.left_image = None
            self.right_image = None
        # Also reset the drone through HAL
        if self.hal_module and hasattr(self.hal_module, 'reset'):
            self.hal_module.reset()


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

# Redirect the console
start_console()


# Expose the user functions
def showImage(image):
    gui.setRightImage(image)


def showLeftImage(image):
    gui.setLeftImage(image)
