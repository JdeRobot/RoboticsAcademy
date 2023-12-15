#!/usr/bin/env python

from __future__ import print_function

import time
import threading
import subprocess
import sys
from datetime import datetime
import re
import json
import importlib

from std_srvs.srv import Empty

from gui import GUI, ThreadGUI
from hal import HAL
from console import start_console, close_console


class Template:
    # Initialize class variables
    # self.ideal_cycle to run an execution for atleast 1 second
    # self.process for the current running process
    def __init__(self):
        self.measure_thread = None
        self.thread = None
        self.reload = False
        self.stop_brain = False
        self.user_code = ""

        # Time variables
        self.ideal_cycle = 80
        self.measured_cycle = 80
        self.iteration_counter = 0
        self.real_time_factor = 0
        self.frequency_message = {'brain': '', 'gui': '',  'rtf': ''}

        # Initialize the GUI, HAL and Console behind the scenes
        self.hal = HAL()
        self.gui = GUI("0.0.0.0", self.hal)
        self.execute_user_code()

    def parse_code(self, source_code):

        sequential_code, iterative_code = self.seperate_seq_iter(
            source_code)
        return iterative_code, sequential_code

    # Function to seperate the iterative and sequential code
    def seperate_seq_iter(self, source_code):
        if source_code == "":
            return "", ""

        # Search for an instance of while True
        infinite_loop = re.search(
            r'[^ ]while\s*\(\s*True\s*\)\s*:|[^ ]while\s*True\s*:|[^ ]while\s*1\s*:|[^ ]while\s*\(\s*1\s*\)\s*:', source_code)

        # Seperate the content inside while True and the other
        # (Seperating the sequential and iterative part!)
        try:
            start_index = infinite_loop.start()
            iterative_code = source_code[start_index:]
            sequential_code = source_code[:start_index]

            # Remove while True: syntax from the code
            # And remove the the 4 spaces indentation before each command
            iterative_code = re.sub(
                r'[^ ]while\s*\(\s*True\s*\)\s*:|[^ ]while\s*True\s*:|[^ ]while\s*1\s*:|[^ ]while\s*\(\s*1\s*\)\s*:', '', iterative_code)
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

        # Reference Environment for the exec() function
        iterative_code, sequential_code = self.parse_code(source_code)

        # Whatever the code is, first step is to just stop!
        self.hal.motors.sendV(0)
        self.hal.motors.sendW(0)

        # print("The debug level is " + str(debug_level)
        # print(sequential_code)
        # print(iterative_code)

        # The Python exec function
        # Run the sequential part
        gui_module, hal_module = self.generate_modules()
        reference_environment = {"GUI": gui_module, "HAL": hal_module}
        exec(sequential_code, reference_environment)

        # Run the iterative part inside template
        # and keep the check for flag
        while self.reload == False:
            while (self.stop_brain == True):
                if (self.reload == True):
                    break
                time.sleep(0.1)

            start_time = datetime.now()

            # Execute the iterative portion
            exec(iterative_code, reference_environment)

            # Template specifics to run!
            finish_time = datetime.now()
            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * \
                1000 + dt.microseconds / 1000.0

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
        hal_module = importlib.util.module_from_spec(
            importlib.machinery.ModuleSpec("HAL", None))
        hal_module.HAL = importlib.util.module_from_spec(
            importlib.machinery.ModuleSpec("HAL", None))
        hal_module.HAL.motors = importlib.util.module_from_spec(
            importlib.machinery.ModuleSpec("motors", None))

        # Add HAL functions
        hal_module.HAL.getPose3d = self.hal.pose3d.getPose3d
        hal_module.HAL.getFrontLaserData = self.hal.laser_front.getLaserData
        hal_module.HAL.getRightLaserData = self.hal.laser_right.getLaserData
        hal_module.HAL.getBackLaserData = self.hal.laser_back.getLaserData
        hal_module.HAL.setV = self.hal.motors.sendV
        hal_module.HAL.setW = self.hal.motors.sendW

        # Define GUI module
        gui_module = importlib.util.module_from_spec(
            importlib.machinery.ModuleSpec("GUI", None))
        gui_module.GUI = importlib.util.module_from_spec(
            importlib.machinery.ModuleSpec("GUI", None))

        # Add GUI functions
        gui_module.GUI.map = self.gui.map

        # Adding modules to system
        # Protip: The names should be different from
        # other modules, otherwise some errors
        sys.modules["HAL"] = hal_module
        sys.modules["GUI"] = gui_module

        return gui_module, hal_module

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
            gui_frequency = round(1000 / self.thread_gui.measured_cycle, 1)
        except ZeroDivisionError:
            gui_frequency = 0

        self.frequency_message["brain"] = brain_frequency
        self.frequency_message["gui"] = gui_frequency
        self.frequency_message["rtf"] = self.real_time_factor

        message = "#freq" + json.dumps(self.frequency_message)

    def track_stats(self):
        args = ["gz", "stats", "-p"]
        # Prints gz statistics. "-p": Output comma-separated values containing-
        # real-time factor (percent), simtime (sec), realtime (sec), paused (T or F)
        stats_process = subprocess.Popen(args, stdout=subprocess.PIPE)
        # bufsize=1 enables line-bufferred mode (the input buffer is flushed
        # automatically on newlines if you would write to process.stdin )
        with stats_process.stdout:
            for line in iter(stats_process.stdout.readline, b''):
                stats_list = [x.strip() for x in line.split(b',')]
                self.real_time_factor = stats_list[0].decode("utf-8")

    # Function to read and set frequency from incoming message

    def read_frequency_message(self, message):
        frequency_message = json.loads(message)

        # Set brain frequency
        frequency = float(frequency_message["brain"])
        self.ideal_cycle = 1000.0 / frequency

        # Set gui frequency
        frequency = float(frequency_message["gui"])
        self.thread_gui.ideal_cycle = 1000.0 / frequency

        return

    def execute_user_code(self):
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
