import os
import json
import cv2
import numpy as np
import base64
import threading
import time
from datetime import datetime
import websocket
import subprocess
import logging
import matplotlib.pyplot as plt
from shared.image import SharedImage
from interfaces.pose3d import ListenerPose3d

from map import Map

# Matrix colors
red = [0, 0, 255]
orange = [0, 165, 255]
yellow = [0, 255, 255]
green = [0, 255, 0]
blue = [255, 0, 0]
indigo = [130, 0, 75]
violet = [211, 0, 148]

# Graphical User Interface Class
class GUI:
    # Initialization function
    # The actual initialization
    def __init__(self, host, hal):
      

        self.payload = {'map': '', 'nav': ''}
        self.client = None
        self.user_mat = None
        self.show_mat = False
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)

        self.host = host

        self.acknowledge = False
        self.acknowledge_lock = threading.Lock()

        self.shared_image = SharedImage("guiimage")

        self.hal = hal
  

        # Create the lap object
        pose3d_object = ListenerPose3d("/gazebo/model_states")
        self.map = Map(pose3d_object)
        self.client_thread = threading.Thread(target=self.run_websocket)
        self.client_thread.start()

    def run_websocket(self):
        while True:
            self.client = websocket.WebSocketApp('ws://127.0.0.1:2303',
                                                 on_message=self.on_message,)
            self.client.run_forever(ping_timeout=None, ping_interval=0)



    # Explicit initialization function
    # Class method, so user can call it without instantiation
    @classmethod
    def initGUI(cls, host, console):
        # self.payload = {'image': '', 'shape': []}
        new_instance = cls(host, console)
        return new_instance


    # Function to get value of Acknowledge
    def get_acknowledge(self):
        self.acknowledge_lock.acquire()
        acknowledge = self.acknowledge
        self.acknowledge_lock.release()

        return acknowledge

    # Function to get value of Acknowledge
    def set_acknowledge(self, value):
        self.acknowledge_lock.acquire()
        self.acknowledge = value
        self.acknowledge_lock.release()

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
    
    def process_colors(self, image):
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
    def get_message(self, client, server, message):
        # Acknowledge Message for GUI Thread
        if(message[:4] == "#ack"):
            self.set_acknowledge(True)

    # load the image data
    def showNumpy(self, image):
        self.shared_image.add(self.process_colors(image))

    def getMap(self, url):        
        return plt.imread(url)

    # Function to reset
    def reset_gui(self):
        self.map.reset()
        self.user_mat = None

    def on_message(self, ws, message):
        """Handles incoming messages from the websocket client."""
        if message.startswith("#ack"):
            self.set_acknowledge(True)


# This class decouples the user thread
# and the GUI update thread
class ThreadGUI:
    def __init__(self, gui):
        self.gui = gui
        self.ideal_cycle = 80
        self.real_time_factor = 0
        self.frequency_message = {'brain': '', 'gui': '', 'rtf': ''}
        self.iteration_counter = 0
        self.running = True

    # Function to start the execution of threads
    def start(self):
        """Starts the GUI, frequency measurement, and real-time factor threads."""
        self.frequency_thread = threading.Thread(target=self.measure_and_send_frequency)
        self.gui_thread = threading.Thread(target=self.run)
        self.rtf_thread = threading.Thread(target=self.get_real_time_factor)
        self.frequency_thread.start()
        self.gui_thread.start()
        self.rtf_thread.start()
        print("GUI Thread Started!")

    def get_real_time_factor(self):
        """Continuously calculates the real-time factor."""
        while True:
            time.sleep(2)
            args = ["gz", "stats", "-p"]
            stats_process = subprocess.Popen(args, stdout=subprocess.PIPE)
            with stats_process.stdout:
                for line in iter(stats_process.stdout.readline, b''):
                    stats_list = [x.strip() for x in line.split(b',')]
                    self.real_time_factor = stats_list[0].decode("utf-8")

    def measure_and_send_frequency(self):
        """Measures and sends the frequency of GUI updates and brain cycles."""
        previous_time = datetime.now()
        while self.running:
            time.sleep(2)
            current_time = datetime.now()
            dt = current_time - previous_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            previous_time = current_time
            measured_cycle = ms / self.iteration_counter if self.iteration_counter > 0 else 0
            self.iteration_counter = 0
            brain_frequency = round(1000 / measured_cycle, 1) if measured_cycle != 0 else 0
            gui_frequency = round(1000 / self.ideal_cycle, 1)
            self.frequency_message = {'brain': brain_frequency, 'gui': gui_frequency, 'rtf': self.real_time_factor}
            message = json.dumps(self.frequency_message)
            if self.gui.client:
                try:
                    self.gui.client.send(message)
                except Exception as e:
                    print(f"Error sending frequency message: {e}")



    # The main thread of execution
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