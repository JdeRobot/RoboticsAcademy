import json
import cv2
import base64
import threading
import time

from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from src.manager.ram_logging.log_manager import LogManager
from console import start_console
import numpy as np


# Graphical User Interface Class

class GUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # Execution control vars
        self.image = None
        self.show_image_request = False
        self.image_lock = threading.Lock()
        self.host = host
        self.payload = {"image": ""}
        self.frame_rgb = None

        self.start()

    # Process incoming messages to the GUI
    def gui_in_thread(self, ws, message):
        time_frame_size = 20

        if "pick" in message:
            base64_buffer = message[4:-time_frame_size]
            time = message[-time_frame_size:]

            if base64_buffer.startswith('data:image/jpeg;base64,'):
                base64_buffer = base64_buffer[len('data:image/jpeg;base64,'):]

            # Decode base64 string to bytes
            image_data = base64.b64decode(base64_buffer)

            # Convert bytes to a numpy array
            nparr = np.frombuffer(image_data, np.uint8)

            # Decode the image (convert it to OpenCV format)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            with self.image_lock:
                self.frame_rgb = img
                ack_message = {'ack_img': 'ack','time':time}
                self.send_to_client(json.dumps(ack_message))

    # Prepares and sends a map to the websocket server
    def update_gui(self):
        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)  

        message = json.dumps(self.payload)
        self.send_to_client(message)

    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadImage(self):
        with self.image_lock:
            show_image_request = self.show_image_request
            image = self.image

        payload = {'image': '', 'shape': ''}
        if show_image_request:
            frame = cv2.imencode('.JPEG', image)[1]
            encoded_image = base64.b64encode(frame)

            payload['image'] = encoded_image.decode('utf-8')
            payload['shape'] = image.shape

            with self.image_lock:
                self.show_image_request = False

        return payload

    # Functions for student to call
    def showImage(self, image):
        with self.image_lock:
            self.image = image
            self.show_image_request = True

    def getImage(self):
        if (self.frame_rgb is None):
            self.frame_rgb = np.ones((240, 320, 3), dtype="uint8") * 255  # Blanco

        return self.frame_rgb

host = "ws://127.0.0.1:2303"
gui = GUI(host)

# Redirect the console
start_console()


# Expose the user functions
def showImage(image):
    gui.showImage(image)


def getImage():
    return gui.getImage()
