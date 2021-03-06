import json
import cv2
import base64
import threading
import time
import numpy as np
from datetime import datetime
from websocket_server import WebsocketServer
import multiprocessing
import logging

from interfaces.pose3d import ListenerPose3d
from shared.image import SharedImage

from lap import Lap
from map import Map

# Graphical User Interface Class
class GUI:
    # Initialization function
    # The actual initialization
    def __init__(self, host, console, hal):
        t = threading.Thread(target=self.run_server)
        
        self.payload = {'image': '','lap': '', 'map': '', 'v':'','w':'', 'text_buffer': ''}
        self.server = None
        self.client = None
        
        self.host = host

        # Image variable
        self.shared_image = SharedImage()
        
        # Take the console object to set the same websocket and client
        self.console = console
        self.hal = hal
        t.start()
        
        # Create the lap object
        pose3d_object = ListenerPose3d("/F1ROS/odom")
        self.lap = Lap(pose3d_object)
        self.map = Map(pose3d_object)

        # Event objects for multiprocessing
        self.ack_event = multiprocessing.Event()
        self.cli_event = multiprocessing.Event()
        self.upd_event = multiprocessing.Event()

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

    # Function to get the client
    # Called when a new client is received
    def get_client(self, client, server):
        self.client = client
        self.console.set_websocket(self.server, self.client)
        self.cli_event.set()
        
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

        # Payload V Message
        v_message = str(self.hal.motors.data.vx)
        self.payload["v"] = v_message

        # Payload W Message
        w_message = str(self.hal.motors.data.az)
        self.payload["w"] = w_message

        # Payload Console Messages
        message_buffer = self.console.get_text_to_be_displayed()
        self.payload["text_buffer"] = json.dumps(message_buffer)
        
        message = "#gui" + json.dumps(self.payload)
        self.server.send_message(self.client, message)
            
    # Function to read the message from websocket
    # Gets called when there is an incoming message from the client
    def get_message(self, client, server, message):
        # Acknowledge Message for GUI Thread
        if(message[:4] == "#ack"):
            # Set acknowledgement flag
            self.ack_event.set()
            
        # Message for Console
        elif(message[:4] == "#con"):
            self.console.prompt(message)

    # Activate the server
    def run_server(self):
        self.server = WebsocketServer(port=2303, host=self.host)
        self.server.set_fn_new_client(self.get_client)
        self.server.set_fn_message_received(self.get_message)
        self.server.run_forever()

    # Function to reset
    def reset_gui(self):
        self.lap.reset()
        self.map.reset()

    # Thread function for event communication with GUIProcess
    def start_event_thread(self):
        thread = threading.Thread(target=self.event_thread)
        thread.start()

        # Return event objects
        return self.ack_event, self.cli_event, self.upd_event

    # The event thread
    def event_thread(self):
        # Thread Loop to wait for update_gui event
        while True:
            self.upd_event.wait()
            if(self.upd_event.is_set()):
                self.update_gui()
                self.upd_event.clear()

        

# This class decouples the user thread
# and the GUI update thread
class ProcessGUI(multiprocessing.Process):
    def __init__(self, events, ideal_cycle, time_cycle):
        super(ProcessGUI, self).__init__()

        # Events
        self.ack_event = events[0]
        self.cli_event = events[1]
        self.upd_event = events[2]

        self.exit_signal = multiprocessing.Event()

        # Time variables
        self.time_cycle = time_cycle
        self.ideal_cycle = ideal_cycle
        self.iteration_counter = 0

    # Function to start the execution of threads
    def run(self):
        # Wait for client before starting
        self.cli_event.wait()

        self.measure_thread = threading.Thread(target=self.measure_thread)
        self.thread = threading.Thread(target=self.run_gui)

        self.measure_thread.start()
        self.thread.start()

        print("GUI Process Started!")

        self.exit_signal.wait()

    # The measuring thread to measure frequency
    def measure_thread(self):
        previous_time = datetime.now()
        while(True):
            # Sleep for 2 seconds
            time.sleep(2)

            # Measure the current time and subtract from previous time to get real time interval
            current_time = datetime.now()
            dt = current_time - previous_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            previous_time = current_time

            # Get the time period
            try:
                # Division by zero
                with self.ideal_cycle.get_lock():
                    self.ideal_cycle.value = ms / self.iteration_counter
            except:
                with self.ideal_cycle.get_lock():
                    self.ideal_cycle.value = 0

            # Reset the counter
            self.iteration_counter = 0

    # The main thread of execution
    def run_gui(self):
        while(True):
            start_time = datetime.now()
            # Send update signal
            self.upd_event.set()

            # Wait for acknowldege signal
            self.ack_event.wait()
            self.ack_event.clear()
            
            finish_time = datetime.now()
            self.iteration_counter = self.iteration_counter + 1
            
            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            
            with self.time_cycle.get_lock():
                time_cycle = self.time_cycle.value

            if(ms < time_cycle):
                time.sleep((time_cycle-ms) / 1000.0)

        self.exit_signal.set()
