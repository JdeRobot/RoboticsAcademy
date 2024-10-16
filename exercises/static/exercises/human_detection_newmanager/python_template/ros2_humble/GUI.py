import json
import os
import rclpy
import cv2
import sys
import base64
import threading
import time
import numpy as np
from datetime import datetime
import websocket
import subprocess
import logging

from hal_interfaces.general.odometry import OdometryNode
from console_interfaces.general.console import start_console

# Graphical User Interface Class
class GUI:
    # Initialization function
    # The actual initialization
    def __init__(self, host):

        self.payload = {'image': '', 'shape': []}
        
        # ROS2 init
        if not rclpy.ok():
            rclpy.init(args=None)
	

	# Image variables
        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()
        self.host = host
        self.client = None

  

        self.ack = False
        self.ack_lock = threading.Lock()
        
        # Create the lap object
        # TODO: maybe move this to HAL and have it be hybrid
       

        self.client_thread = threading.Thread(target=self.run_websocket)
        self.client_thread.start()

    def run_websocket(self):
        while True:
            print("GUI WEBSOCKET CONNECTED")
            self.client = websocket.WebSocketApp(self.host, on_message=self.on_message)
            self.client.run_forever(ping_timeout=None, ping_interval=0)

    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image_to_be_shown = self.image_to_be_shown

        image = image_to_be_shown
        payload = {'image': '', 'shape': ''}

        if not image_to_be_shown_updated:
            return payload

        shape = image.shape
        frame = cv2.imencode('.JPEG', image)[1]
        encoded_image = base64.b64encode(frame)

        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape
        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload

    # Function for student to call
    def showImage(self, image):
        with self.image_show_lock:
            self.image_to_be_shown = image
            self.image_to_be_shown_updated = True

    # Update the gui
    def update_gui(self):
        # print("GUI update")
        # Payload Image Message
        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)
        
        
        message = json.dumps(self.payload)
        if self.client:
            try:
                self.client.send(message)
                # print(message)
            except Exception as e:
                print(f"Error sending message: {e}")

    def on_message(self, ws, message):
        """Handles incoming messages from the websocket client."""
        if message.startswith("#ack"):
            # print("on message" + str(message))
            self.set_acknowledge(True)

    def get_acknowledge(self):
        """Gets the acknowledge status."""
        with self.ack_lock:
            ack = self.ack

        return ack

    def set_acknowledge(self, value):
        """Sets the acknowledge status."""
        with self.ack_lock:
            self.ack = value


class ThreadGUI:
    """Class to manage GUI updates and frequency measurements in separate threads."""

    def __init__(self, gui):
        """Initializes the ThreadGUI with a reference to the GUI instance."""
        self.gui = gui
        self.ideal_cycle = 80
        self.real_time_factor = 0
        self.frequency_message = {'brain': '', 'gui': ''}
        self.iteration_counter = 0
        self.running = True

    def start(self):
        """Starts the GUI, frequency measurement, and real-time factor threads."""
        self.frequency_thread = threading.Thread(target=self.measure_and_send_frequency)
        self.gui_thread = threading.Thread(target=self.run)   
        self.frequency_thread.start()
        self.gui_thread.start()
        print("GUI Thread Started!")

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
            self.frequency_message = {'brain': brain_frequency, 'gui': gui_frequency}
            message = json.dumps(self.frequency_message)
            if self.gui.client:
                try:
                    self.gui.client.send(message)
                except Exception as e:
                    print(f"Error sending frequency message: {e}")

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

start_console()

# Spin a thread to keep the interface updated
thread_gui = ThreadGUI(gui_interface)
thread_gui.start()

def showImage(image):
    gui_interface.showImage(image)
    
