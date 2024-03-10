import multiprocessing
import cv2
import numpy as np
import websocket
import base64
import json
from src.manager.ram_logging.log_manager import LogManager
import time

class GUIInterface:

    def __init__(self):
        self.image_lock = multiprocessing.Lock()
        self.key_lock = multiprocessing.Lock()
        self.ack_lock = multiprocessing.Lock()
        self.client_lock = multiprocessing.Lock()

        self.last_img = np.zeros((400, 400, 3), np.uint8)
        self.last_key = 0
        self.ack_received = True

        self.client = None
        self.client_process = None
        self.gui_out_process = None

        self.start_gui_interface()

    def client_process_function(self):
        while True:
            try:
                self.client.run_forever(ping_timeout=None, ping_interval=0)
            except websocket.WebSocketConnectionClosedException:
                LogManager.logger.error("WebSocket connection closed, attempting to reconnect.")
                time.sleep(5)  # Wait a bit before retrying to avoid spamming connection attempts
                self.start_gui_interface()  # Restart the interface to reconnect

    def gui_out_process_function(self):
        while True:
            with self.ack_lock:
                send_msg = self.ack_received

            if send_msg:
                LogManager.logger.info("Hello there, sending msg")

                with self.image_lock:
                    last_img_copy = self.last_img.copy()

                payload = {'image': '', 'shape': ''}
                shape = last_img_copy.shape
                frame = cv2.imencode('.JPEG', last_img_copy)[1]
                encoded_image = base64.b64encode(frame)
                payload['image'] = encoded_image.decode('utf-8')
                payload['shape'] = shape
                message = json.dumps(payload)

                try:
                    if self.client.sock and self.client.sock.connected:
                        with self.client_lock:
                            self.client.send(message)
                        with self.ack_lock:
                            self.ack_received = False
                    else:
                        raise websocket.WebSocketConnectionClosedException("WebSocket is not connected.")
                except Exception as e:
                    print(f"Error sending message: {e}")
            
            time.sleep(1)

    def on_message(self, ws, message):
        LogManager.logger.info("GUI received a msg: " + str(message))
        if message.startswith("#ack"):
            with self.ack_lock:
                self.ack_received = True

    def showImage(self, img):
        with self.image_lock:
            self.last_img = img

    def start_gui_interface(self):
        LogManager.logger.info("Ultra hiper patatoncia")

        self.client = websocket.WebSocketApp('ws://127.0.0.1:2303',
                                             on_message=self.on_message,
                                             on_close=lambda ws: LogManager.logger.info("WebSocket connection closed."),
                                             on_open=lambda ws: LogManager.logger.info("WebSocket connection opened."))

        self.client_process = multiprocessing.Process(target=self.client_process_function)
        self.client_process.start()

        self.gui_out_process = multiprocessing.Process(target=self.gui_out_process_function)
        self.gui_out_process.start()


# Create the interface object
gui_interface = GUIInterface()

# Expose the showImage method
def showImage(img):
    gui_interface.showImage(img)