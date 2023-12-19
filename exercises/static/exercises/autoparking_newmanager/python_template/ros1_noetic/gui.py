import os
import json

import threading
import time
from datetime import datetime
import websocket
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser

from map import Map

# Graphical User Interface Class


class GUI:
    map = None

    # Initialization function
    # The actual initialization
    def __init__(self, host, hal):

        self.payload = {'map': ''}
        self.server = None
        self.client = None

        self.host = host

        self.acknowledge = False
        self.acknowledge_lock = threading.Lock()

        # Take the console object to set the same websocket and client
        self.hal = hal

        # Create the map object
        laser_object_f = ListenerLaser("/F1ROS/laser_f/scan")
        laser_object_r = ListenerLaser("/F1ROS/laser_r/scan")
        laser_object_b = ListenerLaser("/F1ROS/laser_b/scan")
        pose3d_object = ListenerPose3d("/F1ROS/odom")
        self.map = Map(laser_object_f, laser_object_r,
                       laser_object_b, pose3d_object)
        self.client_thread = threading.Thread(target=self.run_websocket)
        self.client_thread.start()

    # Explicit initialization function
    # Class method, so user can call it without instantiation

    def run_websocket(self):
        self.server = websocket.WebSocketApp('ws://127.0.0.1:2303',
                                             on_open=self.on_open,
                                             on_message=self.on_message)
        self.server.run_forever(ping_timeout=None, ping_interval=0)

    @classmethod
    def initGUI(self):
        # self.payload = {'image': '', 'shape': []}
        pass

    # Function to get the client
    # Called when a new client is received
    def on_open(self, ws):
        print('connected')

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
        # Payload Map Message
        map_message = self.map.get_json_data()
        self.payload["map"] = map_message

        message = json.dumps(self.payload)
        self.server.send(message)

    # Function to read the message from websocket
    # Gets called when there is an incoming message from the client
    def on_message(self, ws, message):
        # Acknowledge Message for GUI Thread
        if (message[:4] == "#ack"):
            self.set_acknowledge(True)

    # Function to reset

    def reset_gui(self):
        self.map.reset()


# This class decouples the user thread
# and the GUI update thread
class ThreadGUI:
    def __init__(self, gui):
        self.gui = gui

        # Time variables
        self.ideal_cycle = 80
        self.measured_cycle = 80
        self.iteration_counter = 0
        self.real_time_factor = 0
        self.frequency_message = {'brain': '', 'gui': '',  'rtf': ''}


    # Function to start the execution of threads
    def start(self):
        self.measure_thread_instance = threading.Thread(target=self.measure_thread)
        self.thread = threading.Thread(target=self.run)

        self.measure_thread_instance.start()
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
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * \
                1000 + dt.microseconds / 1000.0
            previous_time = current_time

            # Get the time period
            try:
                # Division by zero
                self.measured_cycle = ms / self.iteration_counter
            except:
                self.measured_cycle = 0

            # Reset the counter
            self.iteration_counter = 0

        # Function to measure the frequency of iterations
    def measure_frequency(self):
        previous_time = datetime.now()
        # An infinite loop
        while True:
            # Sleep for 2 seconds
            time.sleep(2)

            # Measure the current time and subtract from the previous time to get real time interval
            current_time = datetime.now()
            dt = current_time - previous_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * \
                1000 + dt.microseconds / 1000.0
            previous_time = current_time

            # Get the time period
            try:
                # Division by zero
                self.measured_cycle = ms / self.iteration_counter
            except:
                self.measured_cycle = 0

            # Reset the counter
            self.iteration_counter = 0

            # Send to client
            self.send_frequency_message()

    # Function to generate and send frequency messages
    def send_frequency_message(self):
        # This function generates and sends frequency measures of the brain and gui
        brain_frequency = 0
        gui_frequency = 0
        try:
            brain_frequency = round(1000 / self.measured_cycle, 1)
        except ZeroDivisionError:
            brain_frequency = 0

        try:
            gui_frequency = round(1000 / self.measured_cycle, 1)
        except ZeroDivisionError:
            gui_frequency = 0

        self.frequency_message["brain"] = brain_frequency
        self.frequency_message["gui"] = gui_frequency
        self.frequency_message["rtf"] = self.real_time_factor

        message = json.dumps(self.frequency_message)
        if self.gui.server:
            self.gui.server.send(message)
    
    # The main thread of execution
    def run(self):
        while (self.gui.server == None):
            pass

        while (True):
            start_time = datetime.now()
            self.gui.update_gui() 
            self.send_frequency_message()
            acknowledge_message = self.gui.get_acknowledge()

            while (acknowledge_message == False):
                acknowledge_message = self.gui.get_acknowledge()

            self.gui.set_acknowledge(False)

            finish_time = datetime.now()
            self.iteration_counter = self.iteration_counter + 1

            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * \
                1000 + dt.microseconds / 1000.0
            if (ms < self.ideal_cycle):
                time.sleep((self.ideal_cycle-ms) / 1000.0)
