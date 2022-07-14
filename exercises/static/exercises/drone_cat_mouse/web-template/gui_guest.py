import json
import cv2
import base64
import threading
import time
from datetime import datetime
from websocket_server import WebsocketServer
import logging
import numpy as np
from hal_guest import HAL

# Graphical User Interface Class
class GUI:
    # Initialization function
    # The actual initialization
    def __init__(self, host, hal):
        t = threading.Thread(target=self.run_server)
        
        self.payload = {'image': ''}
        self.left_payload = {'image': ''}
        self.dist = {'dist': '', 'ready': ''}
        self.server = None
        self.client = None
        
        self.host = host

        # Image variables
        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.left_image_to_be_shown = None
        self.left_image_to_be_shown_updated = False
        self.left_image_show_lock = threading.Lock()
        
        self.acknowledge = False
        self.acknowledge_lock = threading.Lock()
        
        # Take the console object to set the same websocket and client
        self.hal = hal
        t.start()

    # Explicit initialization function
    # Class method, so user can call it without instantiation
    @classmethod
    def initGUI(cls, host):
        # self.payload = {'image': '', 'shape': []}
        new_instance = cls(host)
        return new_instance

    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadImage(self):
        self.image_show_lock.acquire()
        image_to_be_shown_updated = self.image_to_be_shown_updated
        image_to_be_shown = self.image_to_be_shown
        self.image_show_lock.release()

        image = image_to_be_shown
        payload = {'image': '', 'shape': ''}

        if not image_to_be_shown_updated:
            return payload

        shape = image.shape
        frame = cv2.imencode('.JPEG', image)[1]
        encoded_image = base64.b64encode(frame)

        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape

        self.image_show_lock.acquire()
        self.image_to_be_shown_updated = False
        self.image_show_lock.release()

        return payload

    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadLeftImage(self):
        self.left_image_show_lock.acquire()
        left_image_to_be_shown_updated = self.left_image_to_be_shown_updated
        left_image_to_be_shown = self.left_image_to_be_shown
        self.left_image_show_lock.release()

        image = left_image_to_be_shown
        payload = {'image': '', 'shape': ''}

        if not left_image_to_be_shown_updated:
            return payload

        shape = image.shape
        frame = cv2.imencode('.JPEG', image)[1]
        encoded_image = base64.b64encode(frame)

        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape

        self.left_image_show_lock.acquire()
        self.left_image_to_be_shown_updated = False
        self.left_image_show_lock.release()

        return payload

    # Function for student to call
    def showImage(self, image):
        self.image_show_lock.acquire()
        self.image_to_be_shown = image
        self.image_to_be_shown_updated = True
        self.image_show_lock.release()

    # Function for student to call
    def showLeftImage(self, image):
        self.left_image_show_lock.acquire()
        self.left_image_to_be_shown = image
        self.left_image_to_be_shown_updated = True
        self.left_image_show_lock.release()

    # Function to get the client
    # Called when a new client is received
    def get_client(self, client, server):
        self.client = client
        
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
        
    # Update the gui
    def update_gui(self):
        # Payload Image Message
        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)
        # Add position to payload
        # mouse_x, mouse_y, mouse_z = self.hal.get_position()
        # self.payload["pos"] = [mouse_x, mouse_y, mouse_z]
        
        message = "#gui" + json.dumps(self.payload)
        self.server.send_message(self.client, message)

        # Payload Left Image Message
        left_payload = self.payloadLeftImage()
        self.left_payload["image"] = json.dumps(left_payload)

        message = "#gul" + json.dumps(self.left_payload)
        self.server.send_message(self.client, message)
            
    # Update distance between cat and mouse drones
    def update_dist(self):
        # cat_x, cat_y, cat_z = self.hal.get_position()
        # mouse_x, mouse_y, mouse_z = self.mouse.get_position()

        # if mouse_z > 0.1: self.dist["ready"] = "true"
        # else: self.dist["ready"] = "false"

        # dist = np.sqrt((mouse_x+2-cat_x)**2 + (mouse_y-cat_y)**2 + (mouse_z-cat_z)**2)
        dist = int(dist*100)/100
        self.dist["dist"] = dist

        message = '#dst' + json.dumps(self.dist)
        self.server.send_message(self.client, message)

    # Function to read the message from websocket
    # Gets called when there is an incoming message from the client
    def get_message(self, client, server, message):
        # Acknowledge Message for GUI Thread
        if message[:4] == "#ack":
            self.set_acknowledge(True)
        # elif message[:4] == "#mou":
        #     self.mouse.start_mouse(int(message[4:5]))
        # elif message[:4] == "#stp":
        #     self.mouse.stop_mouse()
        # elif message[:4] == "#rst":
        #     self.mouse.reset_mouse()

    # Activate the server
    def run_server(self):
        self.server = WebsocketServer(port=2304, host=self.host)
        self.server.set_fn_new_client(self.get_client)
        self.server.set_fn_message_received(self.get_message)

        logged = False
        while not logged:
            try:
                f = open("/ws_gui_code.log", "w")
                f.write("websocket_gui_code=ready")
                f.close()
                logged = True
            except:
                time.sleep(0.1)

        self.server.run_forever()

    # Function to reset
    def reset_gui(self):
        pass
        

# This class decouples the user thread
# and the GUI update thread
class ThreadGUI:
    def __init__(self, gui):
        self.gui = gui

        # Time variables
        self.ideal_cycle = 80
        self.measured_cycle = 80
        self.iteration_counter = 0

    # Function to start the execution of threads
    def start(self):
        self.measure_thread = threading.Thread(target=self.measure_thread)
        self.thread = threading.Thread(target=self.run)

        self.measure_thread.start()
        self.thread.start()

        print("GUI Thread Started!")

    # The measuring thread to measure frequency
    def measure_thread(self):
        while self.gui.client is None:
            pass

        previous_time = datetime.now()
        while True:
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
                self.measured_cycle = ms / self.iteration_counter
            except:
                self.measured_cycle = 0

            # Reset the counter
            self.iteration_counter = 0

    # The main thread of execution
    def run(self):
        while self.gui.client is None:
            pass
    
        while True:
            start_time = datetime.now()
            self.gui.update_gui()
            #self.gui.update_dist()
            acknowledge_message = self.gui.get_acknowledge()
            
            while not acknowledge_message:
                acknowledge_message = self.gui.get_acknowledge()

            self.gui.set_acknowledge(False)
            
            finish_time = datetime.now()
            self.iteration_counter = self.iteration_counter + 1
            
            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            if ms < self.ideal_cycle:
                time.sleep((self.ideal_cycle-ms) / 1000.0)
