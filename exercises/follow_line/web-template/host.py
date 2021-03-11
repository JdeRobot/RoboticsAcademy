#!/usr/bin/env python

from __future__ import print_function

from websocket_server import WebsocketServer
import logging
import time
import threading
import multiprocessing
import sys
from datetime import datetime
import re
import json
import traceback
import imp

import rospy
from std_srvs.srv import Empty
import cv2

from gui import GUI, ProcessGUI
from hal import HAL
import console
from brain import BrainProcess

class Template:
    # Initialize class variables
    # self.time_cycle to run an execution for atleast 1 second
    # self.process for the current running process
    def __init__(self):
        self.brain_process = None
        self.reload = multiprocessing.Event()
        
        # Time variables
        self.brain_time_cycle = multiprocessing.Value('d', 80)
        self.brain_ideal_cycle = multiprocessing.Value('d', 80)

        self.frequency_message = {'brain': '', 'gui': ''}

        # GUI variables
        self.gui_time_cycle = multiprocessing.Value('d', 80)
        self.gui_ideal_cycle = multiprocessing.Value('d', 80)
                
        self.server = None
        self.client = None
        self.host = sys.argv[1]

        # Initialize the GUI, HAL and Console behind the scenes
        self.console = console.Console()
        self.hal = HAL()
        self.gui = GUI(self.host, self.console, self.hal)
        
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
    	if(source_code[:5] == "#save"):
    		source_code = source_code[5:]
    		self.save_code(source_code)
    		
    		return "", ""
    	
    	elif(source_code[:5] == "#load"):
    		source_code = source_code + self.load_code()
    		self.server.send_message(self.client, source_code)
    
    		return "", ""

        elif(source_code[:5] == "#resu"):
                restart_simulation = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
                restart_simulation()

                return "", ""

        elif(source_code[:5] == "#paus"):
                pause_simulation = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
                pause_simulation()

                return "", ""
    		
    	elif(source_code[:5] == "#rest"):
    		reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    		reset_simulation()
    		self.gui.reset_gui()
    		return "", ""
    		
    	else:
    		# Get the frequency of operation, convert to time_cycle and strip
    		try:
        		# Get the debug level and strip the debug part
        		debug_level = int(source_code[5])
        		source_code = source_code[12:]
        	except:
        		debug_level = 1
        		source_code = ""
    		
    		source_code = self.debug_parse(source_code, debug_level)
    		# Pause and unpause
    		if(source_code == ""):
    		    self.gui.lap.pause()
    		else:
    		    self.gui.lap.unpause()
    		sequential_code, iterative_code = self.seperate_seq_iter(source_code)

    		return iterative_code, sequential_code
			
        
    # Function to parse code according to the debugging level
    def debug_parse(self, source_code, debug_level):
    	if(debug_level == 1):
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
    
    # Function to maintain thread execution
    def execute_thread(self, source_code):
        # Keep checking until the thread is alive
        # The thread will die when the coming iteration reads the flag
        if(self.brain_process != None):
            while self.brain_process.is_alive():
                pass

        # Turn the flag down, the iteration has successfully stopped!
        self.reload.clear()
        # New thread execution
        code = self.parse_code(source_code)
        pipes = self.start_pipe_thread()
        self.brain_process = BrainProcess(code, pipes, self.brain_ideal_cycle,
                                          self.brain_time_cycle, self.reload)
        self.brain_process.start()
        print("New Brain Process Started!")

    # Function to start pipe thread for communication with BrainProcess
    def start_pipe_thread(self):
        # Start multiprocessing pipes
        console_pipe1, console_pipe2 = multiprocessing.Pipe()
        hal_pipe1, hal_pipe2 = multiprocessing.Pipe()
        gui_pipe1, gui_pipe2 = multiprocessing.Pipe()

        # Start pipe threads
        console_thread = threading.Thread(target=self.pipe_thread, args=(console_pipe1, self.reload, ))
        hal_thread = threading.Thread(target=self.pipe_thread, args=(hal_pipe1, self.reload, ))
        gui_thread = threading.Thread(target=self.pipe_thread, args=(gui_pipe1, self.reload, ))

        console_thread.start(); hal_thread.start(); gui_thread.start()

        return console_pipe2, hal_pipe2, gui_pipe2

    # The thread function for communication
    def pipe_thread(self, pipe, reload_event):
        while not reload_event.is_set():
            try:
                command = pipe.recv()

                # Exception for showImage()
                if(command == "self.gui.showImage()"):
                    image = pipe.recv()
                    ret_obj = self.gui.showImage(image)
                # Otherwise
                else:
                    exec("ret_obj = " + command)
                pipe.send(ret_obj)
            except EOFError:
                pass

        pipe.close()

    # Function to read and set frequency from incoming message
    def read_frequency_message(self, message):
        frequency_message = json.loads(message)

        # Set brain frequency
        frequency = float(frequency_message["brain"])
        with self.brain_time_cycle.get_lock():
            self.brain_time_cycle.value = 1000.0 / frequency

        # Set gui frequency
        frequency = float(frequency_message["gui"])
        with self.gui_time_cycle.get_lock():
            self.gui_time_cycle.value = 1000.0 / frequency

        return

    # Function to generate and send frequency messages
    def send_frequency_message(self):
        # This function generates and sends frequency measures of the brain and gui
        brain_frequency = 0; gui_frequency = 0
        try:
            with self.brain_ideal_cycle.get_lock():
                brain_frequency = round(1000 / self.brain_ideal_cycle.value, 1)
        except ZeroDivisionError:
            brain_frequency = 0

        try:
            with self.gui_ideal_cycle.get_lock():
                gui_frequency = round(1000 / self.gui_ideal_cycle.value, 1)
        except ZeroDivisionError:
            gui_frequency = 0

        self.frequency_message["brain"] = brain_frequency
        self.frequency_message["gui"] = gui_frequency

        message = "#freq" + json.dumps(self.frequency_message)
        self.server.send_message(self.client, message)


    # The websocket function
    # Gets called when there is an incoming message from the client
    def handle(self, client, server, message):
        if(message[:5] == "#freq"):
            frequency_message = message[5:]
            self.read_frequency_message(frequency_message)
            self.send_frequency_message()
            return
        
        try:
            # Once received turn the reload flag up and send it to execute_thread function
            code = message
            # print(repr(code))
            self.reload.set()
            self.execute_thread(code)
        except:
            pass

    # Function that gets called when the server is connected
    def connected(self, client, server):
    	self.client = client
    	# Start the GUI update thread
        events = self.gui.start_event_thread()
    	self.process_gui = ProcessGUI(events, self.gui_ideal_cycle, self.gui_time_cycle)
    	self.process_gui.start()
        self.hal.start_thread()

        # Initialize the ping message
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
