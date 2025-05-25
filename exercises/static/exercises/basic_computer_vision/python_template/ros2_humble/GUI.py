import json
import cv2
import base64
import threading

from gui_interfaces.general.measuring_threading_gui_no_sim import MeasuringThreadingGUI
from console_interfaces.general.console import start_console
import numpy as np

# Graphical User Interface Class


class GUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # Execution control vars
        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.payload = {"image": "", "shape": ""}
        self.frame_rgb = None
        self.frame_rgb_lock = threading.Lock()

        self.has_received_img = False

        self.start()

    # Process incoming messages to the GUI
    def gui_in_thread(self, ws, message):
        # time frame size
        time_frame_size = 20

        if "ack" in message:
            with self.ack_lock:
                self.ack = True
                self.ack_frontend = True

        if "pick" in message:
            self.has_received_img = True

            # Image from the frontend
            base64_buffer = message[4:-time_frame_size]
            time = message[-time_frame_size:]

            if base64_buffer.startswith("data:image/jpeg;base64,"):
                base64_buffer = base64_buffer[len("data:image/jpeg;base64,") :]

            # Decodificar la cadena base64 a bytes
            image_data = base64.b64decode(base64_buffer)

            # Convertir los bytes a un array de numpy
            nparr = np.frombuffer(image_data, np.uint8)

            # Decodificar la imagen (convertirla a formato OpenCV)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            with self.frame_rgb_lock:
                self.frame_rgb = img
                ack_message = {"ack_img": "ack", "time": time}
                self.send_to_client(json.dumps(ack_message))

        if "introspection" in message:
            info = message[len("introspection:") :]
            self.fps, self.lat = info.split("/")

    # Prepares and sends a map to the websocket server
    def update_gui(self):

        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        message = json.dumps(self.payload)
        self.send_to_client(message)

        if not self.has_received_img:
            ack_message = {"ack_img": "ack", "time": ""}
            self.send_to_client(json.dumps(ack_message))

    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image_to_be_shown = self.image_to_be_shown

        image = image_to_be_shown
        payload = {"image": "", "shape": ""}

        if not image_to_be_shown_updated:
            return payload

        shape = image.shape
        frame = cv2.imencode(".JPEG", image)[1]
        encoded_image = base64.b64encode(frame)

        payload["image"] = encoded_image.decode("utf-8")
        payload["shape"] = shape

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload

    # Function for student to call
    def showImage(self, image):
        with self.image_show_lock:
            self.image_to_be_shown = image
            self.image_to_be_shown_updated = True

    def getImage(self):
        with self.frame_rgb_lock:
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
