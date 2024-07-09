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
from console import start_console

from lap import Lap
from map import Map


# Graphical User Interface Class
class GUI:
    # Initialization function
    # The actual initialization
    def __init__(self, host):

        self.payload = {'image': '','lap': '', 'map': ''}
        
        # ROS2 init
        if not rclpy.ok():
            rclpy.init(args=None)

        # Circuit
        self.circuit = "simple"

        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.host = host
        self.client = None

        self.ack = False
        self.ack_lock = threading.Lock()

        self.iteration_counter = 0
        self.running = True
        
        # Create the lap object
        # TODO: maybe move this to HAL and have it be hybrid
        pose3d_object = OdometryNode("/odom")
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(pose3d_object)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        self.lap = Lap(pose3d_object)
        self.map = Map(pose3d_object, self.circuit)

        # Start the websocket and GUI update threads
        self.websocket_thread = threading.Thread(target=self.run_websocket)
        self.update_thread = threading.Thread(target=self.run)
        self.websocket_thread.start()
        self.update_thread.start()

    def run_websocket(self):
        while self.running:
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

        # Payload Image Message
        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)
        
        # Payload Lap Message
        lapped = self.lap.check_threshold()
        self.payload["lap"] = ""
        if(lapped != None):
            self.payload["lap"] = str(lapped)
            
        # Payload Map Message
        pos_message = str(self.map.getFormulaCoordinates())
        self.payload["map"] = pos_message
        
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

    def run(self):
        """Main loop to update the GUI at regular intervals."""
        while self.running:
            start_time = datetime.now()
            self.update_gui()
            self.iteration_counter += 1
            finish_time = datetime.now()
            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            sleep_time = max(0, (50 - ms) / 1000.0)
            time.sleep(sleep_time)


# Create a GUI interface
host = "ws://127.0.0.1:2303"
gui_interface = GUI(host)

# Start the console
start_console()

def showImage(image):
    gui_interface.showImage(image)