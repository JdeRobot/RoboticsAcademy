#!/usr/bin/env python

from __future__ import print_function
import time
import threading
import sys
from datetime import datetime
import re
import subprocess
import importlib
import os
from GUI import GUI, ThreadGUI
from HAL import HAL
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

    def add_frequency_control(self, code):
        frequency_control_code_imports = """
from datetime import datetime
ideal_cycle = 20
"""
        code = frequency_control_code_imports + code
        infinite_loop = re.search(
            r'[^ ]while\s*\(\s*True\s*\)\s*:|[^ ]while\s*True\s*:|[^ ]while\s*1\s*:|[^ ]while\s*\(\s*1\s*\)\s*:', code)        
        frequency_control_code_pre = """
    start_time = datetime.now()
            """
        code = code[:infinite_loop.end()] + frequency_control_code_pre + code[infinite_loop.end():]
        frequency_control_code_post = """
    finish_time = datetime.now()
    dt = finish_time - start_time
    ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0

    if (ms < ideal_cycle):
        time.sleep((ideal_cycle - ms) / 1000.0)
"""        
        code = code + frequency_control_code_post
        return code


    def process_code(self, source_code):
        """
        Processes the provided source code, executing the sequential part first and then the iterative part.
        """
        start_console()

        self.hal.motors.sendV(0)
        self.hal.motors.sendW(0)

        processed_code = self.add_frequency_control(source_code)
        exec(processed_code)

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
