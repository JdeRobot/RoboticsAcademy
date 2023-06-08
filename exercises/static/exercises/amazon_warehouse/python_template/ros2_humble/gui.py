import json
import os

import rclpy
import cv2
import sys
import numpy as np
import base64
import threading
import time
from datetime import datetime
from websocket_server import WebsocketServer
import multiprocessing
import logging

from interfaces.pose3d import ListenerPose3d
from shared.image import SharedImage
from shared.value import SharedValue

from map import Map


IMG_WIDTH = 320
IMG_HEIGHT = 240

# Graphical User Interface Class
class GUI:
    # Initialization function
    # The actual initialization
    def __init__(self, host):
        rclpy.init()
        rclpy.create_node('GUI')
        self.payload = {'map': '', 'nav': '','image': ''}

        # GUI websocket
        self.server = None
        self.client = None
        self.host = host

        # Create the map object
        pose3d_object = ListenerPose3d("/odom")
        self.map = Map(pose3d_object)

        # Event objects for multiprocessing
        self.ack_event = multiprocessing.Event()
        self.cli_event = multiprocessing.Event()

        # Start server thread
        t = threading.Thread(target=self.run_server)
        t.start()
        
        # Numpy matrix
        self.user_mat = None
        self.show_mat = False

        # Image variable shared with GUIFunctions
        self.shared_image = SharedImage("guiimage")
        # self.shared_numpy = SharedImage("guiNumpy")

#------------------------------------------------------------#

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

    # def loadNumpy(self):
    #     self.user_mat = self.shared_numpy.get()
    #     
    # def showNumpy(self,mat,h,w):
    #     self.user_mat = mat
    #     self.show_mat = True

#------------------------------------------------------------#

    # Update the gui
    def update_gui(self):
        # Payload Map Message
        pos_message = self.map.getRobotCoordinates()
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
        # self.loadNumpy()
        # if (self.show_mat == True):
        #     nav_message = str(self.user_mat.tolist())
        #     self.payload["nav"] = nav_message
        # else:
        #     self.payload["nav"] = None

        # Payload Image Message
        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        message = "#gui" + json.dumps(self.payload)
        self.server.send_message(self.client, message)

    # Function to read the message from websocket
    # Gets called when there is an incoming message from the client
    def get_message(self, client, server, message):
        # Acknowledge Message for GUI Thread
        if(message[:4] == "#ack"):
            # Set acknowledgement flag
            self.ack_event.set()
        # Reset message
        elif(message[:5] == "#rest"):
            self.reset_gui()

    # Function to get the client
    # Called when a new client is received
    def get_client(self, client, server):
        self.client = client
        self.cli_event.set()

        print(client, 'connected')

    # Function that gets called when the connected closes
    def handle_close(self, client, server):
        print(client, 'closed')

    # Activate the server
    def run_server(self):
        self.server = WebsocketServer(port=2303, host=self.host)
        self.server.set_fn_new_client(self.get_client)
        self.server.set_fn_message_received(self.get_message)
        self.server.set_fn_client_left(self.handle_close)

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
class ProcessGUI(multiprocessing.Process):
    def __init__(self):
        super(ProcessGUI, self).__init__()

        self.host = sys.argv[1]
        # Circuit
        self.circuit = sys.argv[2]

        # Time variables
        self.time_cycle = SharedValue("gui_time_cycle")
        self.ideal_cycle = SharedValue("gui_ideal_cycle")
        self.iteration_counter = 0

    # Function to initialize events
    def initialize_events(self):
        # Events
        self.ack_event = self.gui.ack_event
        self.cli_event = self.gui.cli_event
        self.exit_signal = multiprocessing.Event()

    # Function to start the execution of threads
    def run(self):
        # Initialize GUI
        self.gui = GUI(self.host)
        self.initialize_events()

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
                self.ideal_cycle.add(ms / self.iteration_counter)
            except:
                self.ideal_cycle.add(0)

            # Reset the counter
            self.iteration_counter = 0

    # The main thread of execution
    def run_gui(self):
        while(True):
            start_time = datetime.now()
            # Send update signal
            self.gui.update_gui()

            # Wait for acknowldege signal
            self.ack_event.wait()
            self.ack_event.clear()
            
            finish_time = datetime.now()
            self.iteration_counter = self.iteration_counter + 1
            
            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            
            time_cycle = self.time_cycle.get()

            if(ms < time_cycle):
                time.sleep((time_cycle-ms) / 1000.0)

        self.exit_signal.set()

    # Functions to handle auxillary GUI functions
    def reset_gui(self):
        self.gui.reset_gui()


if __name__ == "__main__":
    gui = ProcessGUI()
    gui.start()
