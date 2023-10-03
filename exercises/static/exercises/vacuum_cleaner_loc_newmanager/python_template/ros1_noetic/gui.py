import os
import json
import cv2
import numpy as np
import base64
import threading
import time
from datetime import datetime
from websocket_server import WebsocketServer
import logging
import matplotlib.pyplot as plt

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
        t = threading.Thread(target=self.run_server)

        self.payload = {'map': '', 'nav': ''}
        self.server = None
        self.client = None
        self.user_mat = None
        self.show_mat = False
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)

        self.host = host

        self.acknowledge = False
        self.acknowledge_lock = threading.Lock()

        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.hal = hal
        t.start()

        # Create the lap object
        pose3d_object = ListenerPose3d("/roombaROS/odom")
        self.map = Map(pose3d_object)

    # Explicit initialization function
    # Class method, so user can call it without instantiation
    @classmethod
    def initGUI(cls, host, console):
        # self.payload = {'image': '', 'shape': []}
        new_instance = cls(host, console)
        return new_instance

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

    # encode the image data to be sent to websocket
    def payloadImage(self):

        self.image_show_lock.acquire()
        image_to_be_shown_updated = self.image_to_be_shown_updated
        image_to_be_shown = self.image_to_be_shown
        self.image_show_lock.release()

        image = image_to_be_shown
        payload = {'image': '', 'shape': ''}

        if (image_to_be_shown_updated == False):
            return payload

        colored_image = self.process_colors(image)

        shape = colored_image.shape
        frame = cv2.imencode('.PNG', colored_image)[1]
        encoded_image = base64.b64encode(frame)

        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape

        self.image_show_lock.acquire()
        self.image_to_be_shown_updated = False
        self.image_show_lock.release()

        return payload
    
    def process_colors(self, image):
        colored_image = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
        for i in range(image.shape[0]):
            for j in range(image.shape[1]):
                value = image[i][j]                
                if value < 128: # Grayscale for values < 128
                    colored_image[i][j][0] = value * 2
                    colored_image[i][j][1] = value * 2
                    colored_image[i][j][2] = value * 2
                elif value == 128:  # 128 = red
                    colored_image[i][j][0] = red[0]
                    colored_image[i][j][1] = red[1]
                    colored_image[i][j][2] = red[2]
                elif value == 129:  # 129 = orange
                    colored_image[i][j][0] = orange[0]
                    colored_image[i][j][1] = orange[1]
                    colored_image[i][j][2] = orange[2]
                elif value == 130:  # 130 = yellow
                    colored_image[i][j][0] = yellow[0]
                    colored_image[i][j][1] = yellow[1]
                    colored_image[i][j][2] = yellow[2]
                elif value == 131:  # 131 = green
                    colored_image[i][j][0] = green[0]
                    colored_image[i][j][1] = green[1]
                    colored_image[i][j][2] = green[2]
                elif value == 132:  # 132 = blue
                    colored_image[i][j][0] = blue[0]
                    colored_image[i][j][1] = blue[1]
                    colored_image[i][j][2] = blue[2]
                elif value == 133:  # 133 = indigo
                    colored_image[i][j][0] = indigo[0]
                    colored_image[i][j][1] = indigo[1]
                    colored_image[i][j][2] = indigo[2]
                elif value == 134:  # 134 = violet
                    colored_image[i][j][0] = violet[0]
                    colored_image[i][j][1] = violet[1]
                    colored_image[i][j][2] = violet[2]
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

        message = "#gui" + json.dumps(self.payload)
        self.server.send_message(self.client, message)

    # Function to read the message from websocket
    # Gets called when there is an incoming message from the client
    def get_message(self, client, server, message):
        # Acknowledge Message for GUI Thread
        if(message[:4] == "#ack"):
            self.set_acknowledge(True)

    # load the image data
    def showNumpy(self, image):
        self.image_show_lock.acquire()
        self.image_to_be_shown = image
        self.image_to_be_shown_updated = True
        self.image_show_lock.release()

    def getMap(self):        
        return plt.imread('/RoboticsAcademy/exercises/static/exercises/vacuum_cleaner_loc_newmanager/resources/mapgrannyannie.png')

    # Activate the server
    def run_server(self):
        self.server = WebsocketServer(port=2303, host=self.host)
        self.server.set_fn_new_client(self.get_client)
        self.server.set_fn_message_received(self.get_message)

        home_dir = os.path.expanduser('~')

        logged = False
        while not logged:
            try:
                f = open(f"{home_dir}/ws_gui.log", "w")
                f.write("websocket_gui=ready")
                f.close()
                logged = True
            except:
                time.sleep(0.1)

        self.server.run_forever()

    # Function to reset
    def reset_gui(self):
        self.map.reset()
        self.user_mat = None


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
        while(self.gui.client == None):
            pass

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
                self.measured_cycle = ms / self.iteration_counter
            except:
                self.measured_cycle = 0

            # Reset the counter
            self.iteration_counter = 0

    # The main thread of execution
    def run(self):
        while(self.gui.client == None):
            pass

        while(True):
            start_time = datetime.now()
            self.gui.update_gui()
            acknowledge_message = self.gui.get_acknowledge()

            while(acknowledge_message == False):
                acknowledge_message = self.gui.get_acknowledge()

            self.gui.set_acknowledge(False)

            finish_time = datetime.now()
            self.iteration_counter = self.iteration_counter + 1

            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            if(ms < self.ideal_cycle):
                time.sleep((self.ideal_cycle-ms) / 1000.0)
