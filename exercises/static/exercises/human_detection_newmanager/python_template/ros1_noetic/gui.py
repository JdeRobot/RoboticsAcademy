import json
import os

import rospy
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

from interfaces.pose3d import ListenerPose3d
from shared.image import SharedImage
from shared.image import SharedImage
from shared.value import SharedValue

from lap import Lap
from map import Map

# Graphical User Interface Class
class GUI:
    # Initialization function
    # The actual initialization
    def __init__(self):
        self.payload = {'image': '','lap': '', 'map': '', 'v':'','w':''}

        
        # Circuit
        self.circuit = "simple"

        # Image variable host
        self.shared_image = SharedImage("guiimage")
        
        # Get HAL variables
        self.shared_v = SharedValue("velocity")
        self.shared_w = SharedValue("angular")

        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.acknowledge = False
        self.acknowledge_lock = threading.Lock()
        
        # Create the lap object
        pose3d_object = ListenerPose3d("/F1ROS/odom")
        self.lap = Lap(pose3d_object)
        self.map = Map(pose3d_object, self.circuit)
        # Guest Position
        pose3d_object_guest = ListenerPose3d("/F1ROSGuest/odom")
        self.lap_guest = Lap(pose3d_object_guest)
        self.map_guest = Map(pose3d_object_guest, self.circuit)


        self.client_thread = threading.Thread(target=self.run_websocket)
        self.client_thread.start()

    def run_websocket(self):
        while True:
            self.client = websocket.WebSocketApp('ws://127.0.0.1:2303',
                                                 on_message=self.on_message,)
            self.client.run_forever(ping_timeout=None, ping_interval=0)


    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadImage(self):
        image = self.shared_image.get()
        payload = {'image': '', 'shape': ''}
    	
        shape = image.shape
        frame = cv2.imencode('.JPEG', image)[1]
        encoded_image = base64.b64encode(frame)
        
        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape
        
        return payload

    # Function for student to call
    def showImage(self, image):
        self.image_show_lock.acquire()
        self.image_to_be_shown = image
        self.image_to_be_shown_updated = True
        self.image_show_lock.release()

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
        
        # Payload Map Message Guest
        pos_message_guest = str(self.map_guest.getFormulaCoordinates())
        self.payload["map_guest"] = pos_message_guest

        # Payload V Message
        v_message = str(self.shared_v.get())
        self.payload["v"] = v_message

        # Payload W Message
        w_message = str(self.shared_w.get())
        self.payload["w"] = w_message
        
        message = json.dumps(self.payload)
        if self.client:
            try:
                self.client.send(message)
            except Exception as e:
                print(f"Error sending message: {e}")
            
    def on_message(self, ws, message):
        """Handles incoming messages from the websocket client."""
        if message.startswith("#ack"):
            self.set_acknowledge(True)

    def get_acknowledge(self):
        """Gets the acknowledge status."""
        self.acknowledge_lock.acquire()
        acknowledge = self.acknowledge
        self.acknowledge_lock.release()
        return acknowledge

    def set_acknowledge(self, value):
        """Sets the acknowledge status."""
        self.acknowledge_lock.acquire()
        self.acknowledge = value
        self.acknowledge_lock.release()

class ThreadGUI:
    """Class to manage GUI updates and frequency measurements in separate threads."""

    def __init__(self, gui):
        """Initializes the ThreadGUI with a reference to the GUI instance."""
        self.gui = gui
        self.ideal_cycle = 80
        self.real_time_factor = 0
        self.frequency_message = {'brain': '', 'gui': '', 'rtf': ''}
        self.iteration_counter = 0
        self.running = True

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

