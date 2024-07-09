import rclpy
import json
import cv2
import base64
import threading
import time
import websocket
import numpy as np
import matplotlib.pyplot as plt
from src.manager.ram_logging.log_manager import LogManager
from console import start_console

class ThreadingGUI:

    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):

        # ROS 2 init
        if not rclpy.ok():
            rclpy.init()

        # Execution control vars
        self.out_period = 1.0 / freq

        self.ack = True
        self.ack_frontend = False
        self.ack_lock = threading.Lock()

        self.running = True

        self.host = host
        self.node = rclpy.create_node("node")

        # Image variables
        self.image1_to_be_shown = None
        self.image2_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.point_to_save = []
        self.point_to_send = []

        self.matching_to_save = []
        self.duplicate_matching = False
        self.matching_to_send = []
        self.paint_matching = "F"

        self.payload = {'img1': '', 'img2': '', 'pts': '', 'match': '', 'p_match': 'F'}

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

        # In this case, incoming msgs can only be acks
        if "ack" in message:
            with self.ack_lock:
                self.ack = True
                self.ack_frontend = True

    # Process outcoming messages from the GUI
    def gui_out_thread(self):
        while self.running:
            start_time = time.time()

            # Check if a new map should be sent
            with self.ack_lock:
                if self.ack:
                    self.update_gui()
                    if self.ack_frontend: 
                        self.ack = False

            # Maintain desired frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, self.out_period - elapsed)
            time.sleep(sleep_time)

    # Prepares and sends a map to the websocket server
    # Update the gui
    def update_gui(self):
        payload1, payload2 = self.payloadImage()
        self.payload["img1"] = json.dumps(payload1)
        self.payload["img2"] = json.dumps(payload2)
        length_point_send = len(self.point_to_send)

        if (length_point_send != 0):
            if (length_point_send > 100):
                self.payload["pts"] = json.dumps(self.point_to_send[0:100])
                del self.point_to_send[0:100]
            else:
                self.payload["pts"] = json.dumps(self.point_to_send)
                del self.point_to_send[0:length_point_send]
        else:
            self.payload["pts"] = json.dumps([])

        length_matching_send = len(self.matching_to_send)
        self.payload["match"] = json.dumps(self.matching_to_send)
        del self.matching_to_send[0:length_matching_send]

        self.payload["p_match"] = self.paint_matching


        # Payload Point Message
        message = json.dumps(self.payload)
        if self.client:
            try:
                self.client.send(message)
            except Exception as e:
                LogManager.logger.info(f"Error sending message: {e}")

    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image1_to_be_shown = self.image1_to_be_shown
            image2_to_be_shown = self.image2_to_be_shown

        image1 = image1_to_be_shown
        payload1 = {'img': ''}
        image2 = image2_to_be_shown
        payload2 = {'img': ''}

        if(image_to_be_shown_updated == False):
            return payload1, payload2

        image1 = cv2.resize(image1, (0, 0), fx=0.50, fy=0.50)
        frame1 = cv2.imencode('.JPEG', image1)[1]
        encoded_image1 = base64.b64encode(frame1)
        payload1['img'] = encoded_image1.decode('utf-8')

        image2 = cv2.resize(image2, (0, 0), fx=0.50, fy=0.50)
        frame2 = cv2.imencode('.JPEG', image2)[1]
        encoded_image2 = base64.b64encode(frame2)
        payload2['img'] = encoded_image2.decode('utf-8')

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload1, payload2

    # Function for student to call
    def showImages(self, image1, image2, paint_matching):
        self.paint_matching = paint_matching
        if paint_matching:
            self.paint_matching = "T"
        else:
            self.paint_matching = "F"
        if (np.all(self.image1_to_be_shown == image1) == False or np.all(self.image2_to_be_shown == image2) == False):
            with self.image_show_lock:
                self.image1_to_be_shown = image1
                self.image2_to_be_shown = image2
                self.image_to_be_shown_updated = True

    # Show new points
    def ShowNewPoints(self, points):
        duplicate_point = False
        for i in range(0, len(points)):
            for j in range(0, len(self.point_to_save)):
                if (self.point_to_save[j] == points[i]):
                    duplicate_point = True
            if (duplicate_point == False):
                self.point_to_save.append(points[i])
                self.point_to_send.append(points[i])
            else:
                duplicate_point = False

    # Show all points
    def ShowAllPoints(self, points):
        number_equal_points = 0
        for i in range(0, len(points)):
            for j in range(0, len(self.point_to_save)):
                if (self.point_to_save[j] == points[i]):
                    number_equal_points += 1

        if (number_equal_points != len(points)):
            self.ClearAllPoints()
            for i in range(0, len(points)):
                self.point_to_save.append(points[i])
                self.point_to_send.append(points[i])

    # Show image matching
    def showImageMatching(self, x1, y1, x2, y2):
        matching = [x1, y1, x2, y2]

        for i in range(0, len(self.matching_to_save)):
            if ((self.matching_to_save[i] == matching) == True):
                self.duplicate_matching = True

        if (self.duplicate_matching == False):
            self.matching_to_save.append(matching)
            self.matching_to_send.append(matching)
        else:
            self.duplicate_matching = False

    # Function to reset
    def ClearAllPoints(self):
        self.point_to_save = []
        self.point_to_send = []
        self.matching_to_save = []
        self.matching_to_send = []
        self.server.send_message(self.client, "#res")

    def reset_gui(self):
        self.ClearAllPoints()

host = "ws://127.0.0.1:2303"
gui = ThreadingGUI(host)

# Redirect the console
start_console()

def showImages(image1, image2, paint_matching):
    gui.showImages(image1, image2, paint_matching)

def ShowNewPoints(points):
    gui.ShowNewPoints(points)

def ShowAllPoints(points):
    gui.ShowAllPoints(points)

def showImageMatching(x1, y1, x2, y2):
    gui.showImageMatching(x1, y1, x2, y2)

def ClearAllPoints():
    gui.ClearAllPoints()