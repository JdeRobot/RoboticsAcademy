import json
import cv2
import math
import base64
import threading
import time
from datetime import datetime
import websocket
import rclpy
import numpy as np
import matplotlib.pyplot as plt
from HAL import getPose3d
from console import start_console
from map import Map
from shared.image import SharedImage
from PIL import Image

# Graphical User Interface Class

# Matrix colors
red = [0, 0, 255]
orange = [0, 165, 255]
yellow = [0, 255, 255]
green = [0, 255, 0]
blue = [255, 0, 0]
indigo = [130, 0, 75]
violet = [211, 0, 148]


class GUI:
    # Initialization function
    # The actual initialization
    def __init__(self, host):

        # ROS2 init
        if not rclpy.ok():
            rclpy.init(args=None)

        self.payload = {'map': '', 'user': ''}
        self.server = None
        self.client = None
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)

        self.host = host

        self.ack = False
        self.ack_lock = threading.Lock()
        self.shared_image = SharedImage("guiimage")

        # Create the lap object
        self.map = Map(getPose3d)

        self.client_thread = threading.Thread(target=self.run_websocket)
        self.client_thread.start()

    # Function to get value of Acknowledge
    def get_acknowledge(self):
        with self.ack_lock:
            ack = self.ack

        return ack

    # Function to get value of Acknowledge
    def set_acknowledge(self, value):
        with self.ack_lock:
            self.ack = value

    # encode the image data to be sent to websocket
    def payloadImage(self):

        image = self.shared_image.get()
        payload = {'image': '', 'shape': ''}
    	
        shape = image.shape
        frame = cv2.imencode('.PNG', image)[1]
        encoded_image = base64.b64encode(frame)
        
        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape
        
        return payload

    # Update the gui
    def update_gui(self):
        # Payload Map Message
        pos_message = self.map.getRobotCoordinates()
        if (pos_message == self.init_coords):
            pos_message = self.start_coords
        ang_message = self.map.getRobotAngle()
        pos_message = str(pos_message + ang_message)
        self.payload["map"] = pos_message

        # Example Payload Navigation Data message (random data)
        # 4 colors supported (0, 1, 2, 3)
        #nav_mat = np.zeros((20, 20), int)
        #nav_mat[2, 1] = 1
        #nav_mat[3, 3] = 2
        #nav_mat[5,9] = 3
        #nav_message = str(nav_mat.tolist())

        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        message = json.dumps(self.payload)
        if self.client:
            try:
                self.client.send(message)
            except Exception as e:
                print(f"Error sending message: {e}")

    # Function to read the message from websocket
    # Gets called when there is an incoming message from the client
    def on_message(self, message):
        # Acknowledge Message for GUI Thread
        if (message[:4] == "#ack"):
            self.set_acknowledge(True)

    def process_colors(self, image):
        # print(image)
        colored_image = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)

        # Grayscale for values < 128
        mask = image < 128
        colored_image[mask] = image[mask][:, None] * 2

        # Color lookup table
        color_table = {
            128: red,
            129: orange,
            130: yellow,
            131: green,
            132: blue,
            133: indigo,
            134: violet
        }

        for value, color in color_table.items():
            mask = image == value
            colored_image[mask] = color

        return colored_image
    
    # load the image data
    def showNumpy(self, image):
        self.shared_image.add(self.process_colors(image))

    def getMap(self, url):
        try:
        # Open with PIL
            with Image.open(url) as img:
                img = img.convert("RGB")
                img_array = np.array(img)
            return img_array
        except Exception as e:
            print(f"Error reading image from {url}: {e}")
            return None

    def run_websocket(self):
        while True:
            self.client = websocket.WebSocketApp(self.host, on_message=self.on_message)
            self.client.run_forever(ping_timeout=None, ping_interval=0)

    # Function to reset
    def reset_gui(self):
        self.map.reset()


class ThreadGUI:
    """Class to manage GUI updates and frequency measurements in separate threads."""

    def __init__(self, gui):
        """Initializes the ThreadGUI with a reference to the GUI instance."""
        self.gui = gui
        self.iteration_counter = 0
        self.running = True

    def start(self):
        """Starts the GUI, frequency measurement, and real-time factor threads."""
        self.gui_thread = threading.Thread(target=self.run)
        self.gui_thread.start()
        print("GUI Thread Started!")

    def run(self):
        """Main loop to update the GUI at regular intervals."""
        while self.running:
            start_time = datetime.now()
            self.gui.update_gui()
            self.iteration_counter += 1
            finish_time = datetime.now()
            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            sleep_time = max(0, (50 - ms) / 1000.0)
            time.sleep(sleep_time)


# Create a GUI interface
host = "ws://127.0.0.1:2303"
gui_interface = GUI(host)

# Spin a thread to keep the interface updated
thread_gui = ThreadGUI(gui_interface)
thread_gui.start()

start_console()

# Expose to the user
def showNumpy(image):
    gui_interface.showNumpy(image)

def getMap(url):        
    return gui_interface.getMap(url)