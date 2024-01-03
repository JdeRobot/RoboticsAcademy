#!/usr/bin/env python

from __future__ import print_function


import time
import threading
import sys
from datetime import datetime
import re
import importlib

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
        self.ideal_cycle = 20

        # Initialize the GUI, HAL and Console behind the scenes
        self.hal = HAL()
        self.gui = GUI("0.0.0.0", self.hal)
        self.execute_user_code()

    # Function to parse the code
    # A few assumptions:
    # 1. The user always passes sequential and iterative codes
    # 2. Only a single infinite loop
    def parse_code(self, source_code):
        """
        Parses the provided source code, separating the sequential and iterative parts.
        """
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
        hal_module.HAL.setV = self.hal.motors.sendV
        hal_module.HAL.setW = self.hal.motors.sendW
        hal_module.HAL.laser = self.hal.laser
        hal_module.HAL.getLaserData = self.hal.laser.getLaserData
        hal_module.HAL.getBumperData = self.hal.bumper.getBumperData

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

