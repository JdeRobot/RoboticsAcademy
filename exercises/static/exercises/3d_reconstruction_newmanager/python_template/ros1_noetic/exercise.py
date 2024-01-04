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
    """
    Main class for executing and controlling user code.
    """

    def __init__(self):
        """
        Initializes the Template class, setting up the environment for user code execution.
        """
        self.measure_thread = None
        self.thread = None
        self.reload = False
        self.stop_brain = False
        self.user_code = ""
        self.ideal_cycle = 20

        self.hal = HAL()
        self.gui = GUI("0.0.0.0", self.hal)
        self.execute_user_code()

    def parse_code(self, source_code):
        """
        Parses the provided source code, separating the sequential and iterative parts.
        """
        sequential_code, iterative_code = self.seperate_seq_iter(source_code)
        return iterative_code, sequential_code

    def seperate_seq_iter(self, source_code):
        """
        Separates the source code into sequential and iterative components.
        """
        if source_code == "":
            return "", ""

        infinite_loop = re.search(
            r'[^ ]while\s*\(\s*True\s*\)\s*:|[^ ]while\s*True\s*:|[^ ]while\s*1\s*:|[^ ]while\s*\(\s*1\s*\)\s*:', source_code)

        try:
            start_index = infinite_loop.start()
            iterative_code = source_code[start_index:]
            sequential_code = source_code[:start_index]

            iterative_code = re.sub(
                r'[^ ]while\s*\(\s*True\s*\)\s*:|[^ ]while\s*True\s*:|[^ ]while\s*1\s*:|[^ ]while\s*\(\s*1\s*\)\s*:', '', iterative_code)
            extra_lines = sequential_code.count('\n')
            while (extra_lines >= 0):
                iterative_code = '\n' + iterative_code
                extra_lines -= 1
            iterative_code = re.sub(r'^[ ]{4}', '', iterative_code, flags=re.M)

        except:
            sequential_code = source_code
            iterative_code = ""

        return sequential_code, iterative_code

    def process_code(self, source_code):
        """
        Processes the provided source code, executing the sequential part first and then the iterative part.
        """
        start_console()

        iterative_code, sequential_code = self.parse_code(source_code)

        self.hal.motors.sendV(0)
        self.hal.motors.sendW(0)

        gui_module, hal_module = self.generate_modules()
        reference_environment = {"GUI": gui_module, "HAL": hal_module}
        exec(sequential_code, reference_environment)

        while self.reload == False:
            while (self.stop_brain == True):
                if (self.reload == True):
                    break
                time.sleep(0.1)

            start_time = datetime.now()

            exec(iterative_code, reference_environment)

            finish_time = datetime.now()
            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0

            if (ms < self.ideal_cycle):
                time.sleep((self.ideal_cycle - ms) / 1000.0)

        close_console()
        print("Current Thread Joined!")

    # Function to generate the modules for use in ACE Editor
    def generate_modules(self):
        # Define HAL module
        hal_module = importlib.util.module_from_spec(importlib.machinery.ModuleSpec("HAL", None))
        hal_module.HAL = importlib.util.module_from_spec(importlib.machinery.ModuleSpec("HAL", None))

        # Add HAL functions
        hal_module.HAL.getImage = self.hal.getImage
        hal_module.HAL.graficToOptical = self.hal.graficToOptical
        hal_module.HAL.backproject = self.hal.backproject
        hal_module.HAL.getCameraPosition = self.hal.getCameraPosition
        hal_module.HAL.project = self.hal.project
        hal_module.HAL.opticalToGrafic = self.hal.opticalToGrafic
        hal_module.HAL.project3DScene = self.hal.project3DScene

        # Define GUI module
        gui_module = importlib.util.module_from_spec(importlib.machinery.ModuleSpec("GUI", None))
        gui_module.GUI = importlib.util.module_from_spec(importlib.machinery.ModuleSpec("GUI", None))

        # Add GUI functions
        gui_module.GUI.showImages = self.gui.showImages
        gui_module.GUI.ShowNewPoints = self.gui.ShowNewPoints
        gui_module.GUI.ShowAllPoints = self.gui.ShowAllPoints
        gui_module.GUI.showImageMatching = self.gui.showImageMatching
        gui_module.GUI.ClearAllPoints = self.gui.ClearAllPoints

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

if __name__ == "__main__":
    server = Template()