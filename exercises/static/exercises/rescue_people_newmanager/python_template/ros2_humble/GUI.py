import json
import cv2
import base64
import threading
import time
import websocket
from src.manager.ram_logging.log_manager import LogManager


class ThreadingGUI:

    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):

        # Execution control vars
        self.out_period = 1.0 / freq
        self.right_image = None
        self.left_image = None
        self.image_lock = threading.Lock()
        self.ack = True
        self.ack_lock = threading.Lock()
        self.running = True

        self.host = host
        self.msg_right = {"image_right": ""}
        self.msg_left = {"image_left": ""}

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
                        if self.left_image != None or self.right_image != None:
                            self.send_image()
                            self.ack = False

            # Maintain desired frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, self.out_period - elapsed)
            time.sleep(sleep_time)

    # Prepares and send image to the websocket server
    def send_images(self):

        # Codify the images
        if self.left_image != None:
            _, jpg_left_img = cv2.imencode(".JPEG", self.left_image)
            b64_left_img = base64.b64encode(jpg_left_img).decode("utf-8")
        else:
            b64_left_img = None

        if self.right_image != None:
            _, jpg_right_img = cv2.imencode(".JPEG", self.right_image)
            b64_right_img = base64.b64encode(jpg_right_img).decode("utf-8")
        else:
            b64_right_img = None

        payload = {
            "image_left": b64_left_img,
            "shape_left": self.left_image.shape,
            "image_right": b64_right_img,
            "shape_right": self.right_image.shape,
        }
        self.msg["image"] = json.dumps(payload)
        message = json.dumps(self.msg)
        try:
            if self.client:
                self.client.send(message)
        except Exception as e:
            LogManager.logger.info(f"Error sending message: {e}")

    # Functions to set the next image to be sent
    def setLeftImage(self, image):
        with self.image_lock:
            self.left_image = image

    def setRightImage(self, image):
        with self.image_lock:
            self.right_image = image


host = "ws://127.0.0.1:2303"
gui = ThreadingGUI(host)


# Expose the functions that the users need to use
def showImage(image):
    gui.setRightImage(image)


def showLeftImage(image):
    gui.setLeftImage(image)
