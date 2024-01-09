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
        self.gui = GUI("0.0.0.0")
        self.execute_user_code()

    def execute_user_code(self):
        """
        Executes the provided user code, initializing the graphical interface and processing the code.
        """
        start_console()
        
        with open('/workspace/code/academy.py', 'r') as code_file:
            source_code = code_file.read()
        
        self.thread_gui = ThreadGUI(self.gui)
        self.thread_gui.start()

        exec(source_code)
        print("Código del usuario en ejecución.")

        close_console()

if __name__ == "__main__":
    server = Template()