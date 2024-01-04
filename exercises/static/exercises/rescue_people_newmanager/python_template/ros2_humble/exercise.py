#!/usr/bin/env python

from __future__ import print_function

from websocket_server import WebsocketServer
import time
import threading
import subprocess
import sys
from datetime import datetime
import re
import json
import importlib

import os

import rclpy
from std_srvs.srv import Empty

from gui import GUI, ThreadGUI
from hal import HAL
from console import start_console, close_console


class Template:
    # Initialize class variables
    # self.ideal_cycle to run an execution for at least 1 second
    # self.process for the current running process
    def __init__(self):
        self.measure_thread = None
        self.thread = None
        self.reload = False
        self.stop_brain = True
        self.user_code = ""

        # Time variables
        self.ideal_cycle = 80
        self.measured_cycle = 80
        self.iteration_counter = 0
        self.real_time_factor = 0
        self.frequency_message = {'brain': '', 'gui': '', 'rtf': ''}
        

        # Initialize the GUI, HAL and Console behind the scenes
        self.hal = HAL()
        self.gui = GUI("0.0.0.0")
        self.execute_user_code()

    # Function to parse the code
    # A few assumptions:
    # 1. The user always passes sequential and iterative codes
    # 2. Only a single infinite loop
    def parse_code(self, source_code):
        sequential_code, iterative_code = self.seperate_seq_iter(source_code)
        return iterative_code, sequential_code

    # Function to separate the iterative and sequential code
    def seperate_seq_iter(self, source_code):
        if source_code == "":
            return "", ""

        # Search for an instance of while True
        infinite_loop = re.search(r'[^ ]while\s*\(\s*True\s*\)\s*:|[^ ]while\s*True\s*:|[^ ]while\s*1\s*:|[^ ]while\s*\(\s*1\s*\)\s*:', source_code)

        # Separate the content inside while True and the other
        # (Separating the sequential and iterative part!)
        try:
            start_index = infinite_loop.start()
            iterative_code = source_code[start_index:]
            sequential_code = source_code[:start_index]

            # Remove while True: syntax from the code
            # And remove the the 4 spaces indentation before each command
            iterative_code = re.sub(r'[^ ]while\s*\(\s*True\s*\)\s*:|[^ ]while\s*True\s*:|[^ ]while\s*1\s*:|[^ ]while\s*\(\s*1\s*\)\s*:', '', iterative_code)
            # Add newlines to match line on bug report
            extra_lines = sequential_code.count('\n')
            while (extra_lines >= 0):
                iterative_code = '\n' + iterative_code
                extra_lines -= 1
            iterative_code = re.sub(r'^[ ]{4}', '', iterative_code, flags=re.M)

        except:
            sequential_code = source_code
            iterative_code = ""

        return sequential_code, iterative_code

    # The process function
    def process_code(self, source_code):
        # Redirect the information to console
        start_console()

        iterative_code, sequential_code = self.parse_code(source_code)

        # print(sequential_code)
        # print(iterative_code)

        # The Python exec function
        # Run the sequential part
        gui_module, hal_module = self.generate_modules()
        reference_environment = {"GUI": gui_module, "HAL": hal_module}
        self.stop_brain = False
        while (self.stop_brain == True):
            if (self.reload == True):
                return
            time.sleep(0.1)
        exec(sequential_code, reference_environment)
        time.sleep(1)

        # Run the iterative part inside template
        # and keep the check for flag
        while self.reload == False:
            while (self.stop_brain == True):
                if (self.reload == True):
                    return
                time.sleep(0.1)

            start_time = datetime.now()

            # Execute the iterative portion
            exec(iterative_code, reference_environment)

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
            if (ms < self.ideal_cycle):
                time.sleep((self.ideal_cycle - ms) / 1000.0)

        close_console()
        print("Current Thread Joined!")

    # Function to generate the modules for use in ACE Editor
    def generate_modules(self):
        # Define HAL module
        hal_module = importlib.util.module_from_spec(importlib.machinery.ModuleSpec("HAL", None))
        hal_module.HAL = importlib.util.module_from_spec(importlib.machinery.ModuleSpec("HAL", None))
        # hal_module.drone = imp.new_module("drone")
        # motors# hal_module.HAL.motors = imp.new_module("motors")

        # Add HAL functions
        hal_module.HAL.get_frontal_image = self.hal.get_frontal_image
        hal_module.HAL.get_ventral_image = self.hal.get_ventral_image
        hal_module.HAL.get_position = self.hal.get_position
        hal_module.HAL.get_velocity = self.hal.get_velocity
        hal_module.HAL.get_yaw_rate = self.hal.get_yaw_rate
        hal_module.HAL.get_orientation = self.hal.get_orientation
        hal_module.HAL.get_roll = self.hal.get_roll
        hal_module.HAL.get_pitch = self.hal.get_pitch
        hal_module.HAL.get_yaw = self.hal.get_yaw
        hal_module.HAL.get_landed_state = self.hal.get_landed_state
        hal_module.HAL.set_cmd_pos = self.hal.set_cmd_pos
        hal_module.HAL.set_cmd_vel = self.hal.set_cmd_vel
        hal_module.HAL.set_cmd_mix = self.hal.set_cmd_mix
        hal_module.HAL.takeoff = self.hal.takeoff
        hal_module.HAL.land = self.hal.land

        # Define GUI module
        gui_module = importlib.util.module_from_spec(importlib.machinery.ModuleSpec("GUI", None))
        gui_module.GUI = importlib.util.module_from_spec(importlib.machinery.ModuleSpec("GUI", None))

        # Add GUI functions
        gui_module.GUI.showImage = self.gui.showImage
        gui_module.GUI.showLeftImage = self.gui.showLeftImage

        # Adding modules to system
        # Protip: The names should be different from
        # other modules, otherwise some errors
        sys.modules["HAL"] = hal_module
        sys.modules["GUI"] = gui_module

        return gui_module, hal_module

    def execute_user_code(self):
        """
        Executes the provided user code, initializing the graphical interface and processing the code.
        """
        with open('/workspace/code/academy.py', 'r') as code_file:
            source_code = code_file.read()
        
        self.thread_gui = ThreadGUI(self.gui)
        self.thread_gui.start()

        self.thread = threading.Thread(
            target=self.process_code, args=[source_code])
        self.thread.start()
        print("Código del usuario en ejecución.")


# Execute!
if __name__ == "__main__":
    server = Template()
 