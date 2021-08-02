#!/usr/bin/env python

from __future__ import print_function

from websocket_server import WebsocketServer
import logging
import time
import threading
import sys
from datetime import datetime
import re
import json
import traceback
import imp
import numpy as np
import base64

import subprocess
import rospy
from std_srvs.srv import Empty
import cv2
from gui import GUI, ThreadGUI
from hal import HAL
from console import start_console, close_console


class Template:
    # Initialize class variables
    # self.time_cycle to run an execution for atleast 1 second
    # self.process for the current running process
    def __init__(self):
        self.thread = None
        self.reload = False

        # Time variables
        self.time_cycle = 80
        self.ideal_cycle = 80
        self.iteration_counter = 0
        self.real_time_factor = 0
        self.frequency_message = {'brain': '', 'gui': '', 'rtf': ''}

        self.server = None
        self.client = None
        self.host = sys.argv[1]

        # Initialize the GUI, HAL and Console behind the scenes
        self.hal = HAL()
        self.gui = GUI(self.host, self.hal)

    # Function for saving
    def save_code(self, source_code):
        with open('code/academy.py', 'w') as code_file:
            code_file.write(source_code)

    # Function for loading
    def load_code(self):
        with open('code/academy.py', 'r') as code_file:
            source_code = code_file.read()

        return source_code

    # Function to parse the code
    # A few assumptions:
    # 1. The user always passes sequential and iterative codes
    # 2. Only a single infinite loop
    def parse_code(self, source_code):
        # Check for save/load
        if (source_code[:5] == "#save"):
            source_code = source_code[5:]
            self.save_code(source_code)

            return "", "", 1

        elif (source_code[:5] == "#load"):
            source_code = source_code + self.load_code()
            self.server.send_message(self.client, source_code)

            return "", "", 1

        elif (source_code[:5] == "#resu"):
            restart_simulation = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            restart_simulation()

            return "", "", 1

        elif (source_code[:5] == "#paus"):
            pause_simulation = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
            pause_simulation()

            return "", "", 1

        elif (source_code[:5] == "#rest"):
            reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_simulation()
            self.gui.reset_gui()
            return "", ""

        else:
            # Get the frequency of operation, convert to time_cycle and strip

            sequential_code, iterative_code = self.seperate_seq_iter(source_code)
            return iterative_code, sequential_code

    # Function to parse code according to the debugging level
    def debug_parse(self, source_code, debug_level):
        if (debug_level == 1):
            # If debug level is 0, then all the GUI operations should not be called
            source_code = re.sub(r'GUI\..*', '', source_code)

        return source_code

    # Function to seperate the iterative and sequential code
    def seperate_seq_iter(self, source_code):
        if source_code == "":
            return "", ""

        # Search for an instance of while True

        infinite_loop = re.search(r'[^ \t]while\(True\):|[^ \t]while True:', source_code)

        # Seperate the content inside while True and the other
        # (Seperating the sequential and iterative part!)
        try:
            start_index = infinite_loop.start()
            iterative_code = source_code[start_index:]
            sequential_code = source_code[:start_index]

            # Remove while True: syntax from the code
            # And remove the the 4 spaces indentation before each command
            iterative_code = re.sub(r'[^ ]while\(True\):|[^ ]while True:', '', iterative_code)
            iterative_code = re.sub(r'^[ ]{4}', '', iterative_code, flags=re.M)

        except:
            sequential_code = source_code
            iterative_code = ""

        return sequential_code, iterative_code

    # The process function
    def process_code(self, source_code):
        # Redirect the information to console
        start_console()
        # Reference Environment for the exec() function
        iterative_code, sequential_code = self.parse_code(source_code)

        # print("The debug level is " + str(debug_level)
        # print(sequential_code)
        # print(iterative_code)

        # Whatever the code is, first step is to just stop!
        self.hal.motors.sendV(0)
        self.hal.motors.sendW(0)

        # The Python exec function
        # Run the sequential part
        gui_module, hal_module, map_module = self.generate_modules()
        exec(sequential_code, {"GUI": gui_module, "HAL": hal_module, "MAP": map_module})

        # Run the iterative part inside template
        # and keep the check for flag
        while self.reload == False:
            start_time = datetime.now()

            # Execute the iterative portion
            exec(iterative_code, {"GUI": gui_module, "HAL": hal_module, "MAP": map_module})

            # Template specifics to run!
            finish_time = datetime.now()
            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0

            # Keep updating the iteration counter
            if (iterative_code == ""):
                self.iteration_counter = 0
            else:
                self.iteration_counter = self.iteration_counter + 1

            # The code should be run for atleast the target time step
            # If it's less put to sleep
            # If it's more no problem as such, but we can change it!
            if (ms < self.time_cycle):
                time.sleep((self.time_cycle - ms) / 1000.0)

        close_console()
        print("Current Thread Joined!")

    def getMap(self):
        img = cv2.imread("assets/img/cityLargeBin.png", cv2.IMREAD_GRAYSCALE)
        return img

    def getPose(self):
        pose = self.hal.pose3d.getPose3d()
        x = pose.x
        y = pose.y
        rt = pose.yaw
        return [x, y, rt]

    # Function to generate the modules for use in ACE Editor
    def generate_modules(self):
        # Define HAL module

        hal_module = imp.new_module("HAL")
        hal_module.HAL = imp.new_module("HAL")
        hal_module.HAL.motors = imp.new_module("motors")
        hal_module.HAL.getPose = self.getPose

        gui_module = imp.new_module("GUI")
        gui_module.GUI = imp.new_module("GUI")
        # Add GUI functions
        gui_module.GUI.showGPP = self.gui.showGPP
        gui_module.GUI.showPath = self.gui.showPath
        gui_module.GUI.targetPose = self.gui.worldXY
        # Add HAL functions
        hal_module.HAL.motors.sendV = self.hal.motors.sendV
        hal_module.HAL.motors.sendW = self.hal.motors.sendW

        # Define GUI module
        map_module = imp.new_module("MAP")
        map_module.MAP = imp.new_module("MAP")
        map_module.MAP.robotPose = self.gui.update_gui
        map_module.MAP.getGridVal = self.gui.map.getVal
        map_module.MAP.setGridVal = self.gui.map.setVal
        map_module.MAP.gridToWorld = self.gui.map.gridToWorld
        map_module.MAP.worldToGrid = self.gui.map.worldToGrid
        # map_module.MAP.destination = self.gui.mapXY
        map_module.MAP.getMap = self.getMap

        # Adding modules to system
        # Protip: The names should be different from
        # other modules, otherwise some errors
        sys.modules["MAP"] = map_module
        sys.modules["HAL"] = hal_module
        sys.modules["GUI"] = gui_module

        return gui_module, hal_module, map_module

        # Function to measure the frequency of iterations

    def measure_frequency(self):
        previous_time = datetime.now()
        # An infinite loop
        while self.reload == False:
            # Sleep for 2 seconds
            time.sleep(2)

            # Measure the current time and subtract from the previous time to get real time interval
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
            self.send_frequency_message()

    # Function to generate and send frequency messages
    def send_frequency_message(self):
        # This function generates and sends frequency measures of the brain and gui
        brain_frequency = 0
        gui_frequency = 0
        try:
            brain_frequency = round(1000 / self.ideal_cycle, 1)
        except ZeroDivisionError:
            brain_frequency = 0

        try:
            gui_frequency = round(1000 / self.thread_gui.ideal_cycle, 1)
        except ZeroDivisionError:
            gui_frequency = 0

        self.frequency_message["brain"] = brain_frequency
        self.frequency_message["gui"] = gui_frequency
        self.frequency_message["rtf"] = self.real_time_factor

        message = "#freq" + json.dumps(self.frequency_message)
        self.server.send_message(self.client, message)

    # Function to track the real time factor from Gazebo statistics
    # https://stackoverflow.com/a/17698359
    # (For reference, Python3 solution specified in the same answer)
    def track_stats(self):
        args = ["gz", "stats", "-p"]
        # Prints gz statistics. "-p": Output comma-separated values containing-
        # real-time factor (percent), simtime (sec), realtime (sec), paused (T or F)
        stats_process = subprocess.Popen(args, stdout=subprocess.PIPE, bufsize=1)
        # bufsize=1 enables line-bufferred mode (the input buffer is flushed
        # automatically on newlines if you would write to process.stdin )
        with stats_process.stdout:
            for line in iter(stats_process.stdout.readline, b''):
                stats_list = [x.strip() for x in line.split(',')]
                self.real_time_factor = stats_list[0]

    # Function to maintain thread execution
    def execute_thread(self, source_code):
        # Keep checking until the thread is alive
        # The thread will die when the coming iteration reads the flag
        if (self.thread is not None):
            while self.thread.is_alive() or self.measure_thread.is_alive():
                pass

        # Turn the flag down, the iteration has successfully stopped!
        self.reload = False
        # New thread execution
        self.measure_thread = threading.Thread(target=self.measure_frequency)
        self.thread = threading.Thread(target=self.process_code, args=[source_code])
        self.thread.start()
        self.measure_thread.start()
        print("New Thread Started!")

    # Function to read and set frequency from incoming message
    def read_frequency_message(self, message):
        frequency_message = json.loads(message)

        # Set brain frequency
        frequency = float(frequency_message["brain"])
        self.time_cycle = 1000.0 / frequency

        # Set gui frequency
        frequency = float(frequency_message["gui"])
        self.thread_gui.time_cycle = 1000.0 / frequency

        return

    # The websocket function
    # Gets called when there is an incoming message from the client
    def handle(self, client, server, message):
        if (message[:5] == "#freq"):
            frequency_message = message[5:]
            self.read_frequency_message(frequency_message)
            time.sleep(1)
            self.send_frequency_message()
            return

        try:
            # Once received turn the reload flag up and send it to execute_thread function
            code = message
            # print(repr(code))
            self.reload = True
            self.execute_thread(code)
        except:
            pass

    # Function that gets called when the server is connected
    def connected(self, client, server):
        self.client = client
        # Start the GUI update thread
        self.thread_gui = ThreadGUI(self.gui)
        self.thread_gui.start()

        # Start the real time factor tracker thread
        self.stats_thread = threading.Thread(target=self.track_stats)
        self.stats_thread.start()

        # Initialize the frequency message
        self.send_frequency_message()

        print(client, 'connected')

    # Function that gets called when the connected closes
    def handle_close(self, client, server):
        print(client, 'closed')

    def run_server(self):
        self.server = WebsocketServer(port=1905, host=self.host)
        self.server.set_fn_new_client(self.connected)
        self.server.set_fn_client_left(self.handle_close)
        self.server.set_fn_message_received(self.handle)
        self.server.run_forever()


# Execute!
if __name__ == "__main__":
    server = Template()
    server.run_server()
