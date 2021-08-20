import json
import rospy
import cv2
import sys
import base64
import threading
import time
import numpy as np
from datetime import datetime
from websocket_server import WebsocketServer
import logging

# Graphical User Interface Class
class GUI:
    # Initialization function
    # The actual initialization
    def __init__(self, host, hal):
        t = threading.Thread(target=self.run_server)

        self.payload = {'imageL': '', 'imageC': '', 'imageR': '', 'v':'','w':''}
        self.server = None
        self.client = None

        self.host = host

        # Images variable
        self.imageL_to_be_shown = None
        self.imageC_to_be_shown = None
        self.imageR_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        # Get HAL variables
        self.v = None
        self.w = None

        self.pose3d = None

        self.acknowledge = False
        self.acknowledge_lock = threading.Lock()

        # Get HAL Object
        self.hal = hal
        t.start()

    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadImage(self):
        self.image_show_lock.acquire()
        image_to_be_shown_updated = self.image_to_be_shown_updated
        imageL_to_be_shown = self.imageL_to_be_shown
        imageC_to_be_shown = self.imageC_to_be_shown
        imageR_to_be_shown = self.imageR_to_be_shown
        self.image_show_lock.release()

        imageL = imageL_to_be_shown
        payloadL = {'img': ''}
        imageC = imageC_to_be_shown
        payloadC = {'img': ''}
        imageR = imageR_to_be_shown
        payloadR = {'img': ''}

        if(image_to_be_shown_updated == False):
            return payloadL, payloadC, payloadR

        imageL = cv2.resize(imageL, (0, 0), fx=0.50, fy=0.50)
        frameL = cv2.imencode('.JPEG', imageL)[1]
        encoded_imageL = base64.b64encode(frameL)
        payloadL['img'] = encoded_imageL.decode('utf-8')

        imageC = cv2.resize(imageC, (0, 0), fx=0.50, fy=0.50)
        frameC = cv2.imencode('.JPEG', imageC)[1]
        encoded_imageC = base64.b64encode(frameC)
        payloadC['img'] = encoded_imageC.decode('utf-8')

        imageR = cv2.resize(imageR, (0, 0), fx=0.50, fy=0.50)
        frameR = cv2.imencode('.JPEG', imageR)[1]
        encoded_imageR = base64.b64encode(frameR)
        payloadR['img'] = encoded_imageR.decode('utf-8')

        self.image_show_lock.acquire()
        self.image_to_be_shown_updated = False
        self.image_show_lock.release()
        return payloadL, payloadC, payloadR

   # Function for student to call
    def showImages(self, imageL, imageC, imageR):
        if (np.all(self.imageL_to_be_shown == imageL) == False or np.all(self.imageC_to_be_shown == imageC) == False or np.all(self.imageR_to_be_shown == imageR) == False):
            self.image_show_lock.acquire()
            self.imageL_to_be_shown = imageL
            self.imageC_to_be_shown = imageC
            self.imageR_to_be_shown = imageR
            self.image_to_be_shown_updated = True
            self.image_show_lock.release()

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
        payloadL, payloadC, payloadR = self.payloadImage()
        self.payload["imageL"] = json.dumps(payloadL)
        self.payload["imageC"] = json.dumps(payloadC)
        self.payload["imageR"] = json.dumps(payloadR)

        self.payload["v"] = json.dumps(self.v)
        self.payload["w"] = json.dumps(self.w)


        # Payload Point Message
        message = "#gui" + json.dumps(self.payload)
        self.server.send_message(self.client, message)

    # Function to read the message from websocket
    # Gets called when there is an incoming message from the client
    def get_message(self, client, server, message):
        # Acknowledge Message for GUI Thread
        if (message[:4] == "#ack"):
            self.set_acknowledge(True)

    # Activate the server
    def run_server(self):
        self.server = WebsocketServer(port=2303, host=self.host)
        self.server.set_fn_new_client(self.get_client)
        self.server.set_fn_message_received(self.get_message)
        self.server.run_forever()

    def reset_gui(self):
        # Reset Gui
        print("reset")

# This class decouples the user thread
# and the GUI update thread
class ThreadGUI:
    def __init__(self, gui):
        self.gui = gui
        # Time variables
        self.time_cycle = 80
        self.ideal_cycle = 80
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
        while (self.gui.client == None):
            pass

        previous_time = datetime.now()
        while (True):
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
                self.ideal_cycle = ms / self.iteration_counter
            except:
                self.ideal_cycle = 0

            # Reset the counter
            self.iteration_counter = 0

    def run(self):
        while (self.gui.client == None):
            pass

        while (True):
            start_time = datetime.now()
            self.gui.update_gui()
            acknowledge_message = self.gui.get_acknowledge()

            while (acknowledge_message == False):
                acknowledge_message = self.gui.get_acknowledge()

            self.gui.set_acknowledge(False)

            finish_time = datetime.now()
            self.iteration_counter = self.iteration_counter + 1

            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            if (ms < self.time_cycle):
                time.sleep((self.time_cycle - ms) / 1000.0)
