import json
import cv2
import base64
import threading
import time
from datetime import datetime
from websocket_server import WebsocketServer
import logging
import numpy as np


# Graphical User Interface Class
class GUI:
    # Initialization function
    # The actual initialization
    def __init__(self, host, hal):
        t = threading.Thread(target=self.run_server)

        self.payload = {'image': '', 'point': '', 'matching': '', 'paint_matching': ''}
        self.server = None
        self.client = None

        self.host = host

        # Image variables
        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.acknowledge = False
        self.acknowledge_lock = threading.Lock()

        self.point_to_save = []
        self.point_to_send = []

        self.matching_to_save = []
        self.duplicate_matching = False
        self.matching_to_send = []
        self.paint_matching = False

        # Get HAL object
        self.hal = hal
        t.start()

    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadImage(self):
        self.image_show_lock.acquire()
        image_to_be_shown_updated = self.image_to_be_shown_updated
        image_to_be_shown = self.image_to_be_shown
        self.image_show_lock.release()

        image = image_to_be_shown
        payload = {'image': '', 'shape': ''}

        if (image_to_be_shown_updated == False):
            return payload

        shape = image.shape

        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        frame = cv2.imencode('.JPEG', image)[1]
        encoded_image = base64.b64encode(frame)

        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape

        self.image_show_lock.acquire()
        self.image_to_be_shown_updated = False
        self.image_show_lock.release()

        return payload

    # Function for student to call
    def showImage(self, image):
        self.image_show_lock.acquire()
        self.image_to_be_shown = image
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

        # Payload Image Message
        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        # Payload Point Message
        length_point_send = len(self.point_to_send)
        if (length_point_send != 0):
            if (length_point_send > 20):
                self.payload["point"] = json.dumps(self.point_to_send[0:20])
                del self.point_to_send[0:20]
            else:
                self.payload["point"] = json.dumps(self.point_to_send)
                del self.point_to_send[0:length_point_send]
        else:
            self.payload["point"] = json.dumps([])

        length_matching_send = len(self.matching_to_send)
        self.payload["matching"] = json.dumps(self.matching_to_send)
        del self.matching_to_send[0:length_matching_send]

        self.payload["paint_matching"] = self.paint_matching

        message = "#gui" + json.dumps(self.payload)
        print("Payload Enviando el gui")
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
