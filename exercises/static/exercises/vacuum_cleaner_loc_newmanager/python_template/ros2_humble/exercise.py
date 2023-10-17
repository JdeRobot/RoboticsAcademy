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
        self.frequency_message = {'brain': '', 'gui': '', 'rtf': ''}

        self.server = None
        self.client = None
        self.host = sys.argv[1]

        # Initialize the GUI, HAL and Console behind the scenes
        self.hal = HAL()
        self.gui = GUI(self.host, self.hal)

    ################ --- BRAIN --- ################

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
        while not self.reload:
            while (self.stop_brain):
                if (self.reload):
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
        hal_module.HAL.getPose3d = self.hal.getPose3d
        hal_module.HAL.setV = self.hal.setV
        hal_module.HAL.setW = self.hal.setW
        hal_module.HAL.laser = self.hal.laser
        hal_module.HAL.getLaserData = self.hal.getLaserData
        hal_module.HAL.getBumperData = self.hal.getBumperData

        # Define GUI module
        gui_module = importlib.util.module_from_spec(
            importlib.machinery.ModuleSpec("GUI", None))
        gui_module.GUI = importlib.util.module_from_spec(
            importlib.machinery.ModuleSpec("GUI", None))

        # Add GUI functions
        # gui_module.GUI.showImage = self.gui.showImage

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


    ################ --- EXERCISE --- ################

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

            return "", ""

        elif (source_code[:5] == "#load"):
            source_code = source_code + self.load_code()
            self.server.send_message(self.client, source_code)

            return "", ""

        else:
            sequential_code, iterative_code = self.seperate_seq_iter(
                source_code)
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
        self.server.send_message(self.client, message)

    def send_ping_message(self):
        self.server.send_message(self.client, "#ping")

    # Function to notify the front end that the code was received and sent to execution
    def send_code_message(self):
        self.server.send_message(self.client, "#exec")

    # Function to track the real time factor from Gazebo statistics
    # https://stackoverflow.com/a/17698359
    # (For reference, Python3 solution specified in the same answer)
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

    # Function to maintain thread execution
    def execute_thread(self, source_code):
        # Keep checking until the thread is alive
        # The thread will die when the coming iteration reads the flag
        if (self.thread != None):
            while self.thread.is_alive():
                time.sleep(0.2)

        # Turn the flag down, the iteration has successfully stopped!
        self.reload = False
        # New thread execution
        self.thread = threading.Thread(
            target=self.process_code, args=[source_code])
        self.thread.start()
        self.send_code_message()
        print("New Thread Started!")

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

    # The websocket function
    # Gets called when there is an incoming message from the client
    def handle(self, client, server, message):
        if (message[:5] == "#freq"):
            frequency_message = message[5:]
            self.read_frequency_message(frequency_message)
            time.sleep(1)
            self.send_frequency_message()
            return

        elif (message[:5] == "#ping"):
            time.sleep(1)
            self.send_ping_message()
            return

        elif (message[:5] == "#code"):
            try:
                # Once received turn the reload flag up and send it to execute_thread function
                self.user_code = message[6:]
                # print(repr(code))
                self.reload = True
                self.stop_brain = True
                self.execute_thread(self.user_code)
            except:
                pass

        elif (message[:5] == "#rest"):
            try:
                self.reload = True
                self.stop_brain = True
                self.execute_thread(self.user_code)
            except:
                pass

        elif (message[:5] == "#stop"):
            self.stop_brain = True

        elif (message[:5] == "#play"):
            self.stop_brain = False

    # Function that gets called when the server is connected
    def connected(self, client, server):
        self.client = client
        # Start the GUI update thread
        self.thread_gui = ThreadGUI(self.gui)
        self.thread_gui.start()

        # Start the real time factor tracker thread
        self.stats_thread = threading.Thread(target=self.track_stats)
        self.stats_thread.start()

        # Start measure frequency
        self.measure_thread = threading.Thread(target=self.measure_frequency)
        self.measure_thread.start()

        # Initialize the ping message
        # self.send_frequency_message()

        print(client, 'connected')

    # Function that gets called when the connected closes
    def handle_close(self, client, server):
        print(client, 'closed')

    def run_server(self):
        self.server = WebsocketServer(port=1905, host=self.host)
        self.server.set_fn_new_client(self.connected)
        self.server.set_fn_client_left(self.handle_close)
        self.server.set_fn_message_received(self.handle)

        home_dir = os.path.expanduser('~')

        logged = False
        while not logged:
            try:
                f = open(f"{home_dir}/ws_code.log", "w")
                f.write("websocket_code=ready")
                f.close()
                logged = True
            except:
                time.sleep(0.1)

        self.server.run_forever()


# Execute!
if __name__ == "__main__":
    server = Template()
    server.run_server()
