import json
import cv2
import base64
import threading
import time
import websocket
from src.manager.ram_logging.log_manager import LogManager
from console import start_console
import numpy as np


class ThreadingGUI:

    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):

        # Execution control vars
        self.out_period = 1.0 / freq
        self.image = None
        self.image_lock = threading.Lock()
        self.ack = True
        self.ack_lock = threading.Lock()
        self.running = True

        self.host = host
        self.msg = {"image": ""}

        # Initialize and start the WebSocket client thread
        threading.Thread(target=self.run_websocket, daemon=True).start()

        # Initialize and start the image sending thread (GUI out thread)
        threading.Thread(
            target=self.gui_out_thread, name="gui_out_thread", daemon=True
        ).start()

    # Init websocket client
    def run_websocket(self):
        self.client = websocket.WebSocketApp(self.host, on_message=self.gui_in_thread)
        self.client.run_forever(ping_timeout=None, ping_interval=0)

    # Process incoming messages to the GUI
    def gui_in_thread(self, ws, message):

        # In this case, messages can be either acks or key strokes
        if "ack" in message:
            with self.ack_lock:
                self.ack = True
        else:
            LogManager.logger.error("Unsupported msg")

    # Process outcoming messages from the GUI
    def gui_out_thread(self):
        while self.running:
            start_time = time.time()

            # Check if a new image should be sent
            with self.ack_lock:
                with self.image_lock:
                    if self.ack:
                        if np.any(self.image):
                            self.send_image()
                            self.ack = False

            # Maintain desired frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, self.out_period - elapsed)
            time.sleep(sleep_time)

    # Prepares and send image to the websocket server
    def send_image(self):

        if np.any(self.image):
            _, encoded_image = cv2.imencode(".JPEG", self.image)
            b64 = base64.b64encode(encoded_image).decode("utf-8")
            shape = self.image.shape
        else:
            b64 = None
            shape = 0

        payload= {
            "image": b64,
            "shape": shape,
        }

        self.msg["image"] = json.dumps(payload)
        message = json.dumps(self.msg)
        try:
            if self.client:
                self.client.send(message)
        except Exception as e:
            LogManager.logger.info(f"Error sending message: {e}")

    # Functions to set the next image to be sent
    def setImage(self, image):
        with self.image_lock:
            self.image = image


host = "ws://127.0.0.1:2303"
gui = ThreadingGUI(host)


# Redirect the console
start_console()


# Expose the user functions
def showImage(image):
    gui.setImage(image)


