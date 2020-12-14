import json
import cv2
import base64
import threading
import time
from datetime import datetime
from websocket_server import WebsocketServer
import logging

from interfaces.pose3d import ListenerPose3d

from lap import Lap
from map import Map

# Graphical User Interface Class
class GUI:
    # Initialization function
    # The actual initialization
    def __init__(self, host, console, hal):
        t = threading.Thread(target=self.run_server)
        
        self.payload = {'image': '','lap': '', 'map': ''}
        self.server = None
        self.client = None
        
        self.host = host

        self.show_image = False
        self.show_lock = threading.Lock()
        
        self.acknowledge = False
        self.acknowledge_lock = threading.Lock()
        
        # Take the console object to set the same websocket and client
        self.console = console
        self.hal = hal
        t.start()
        
        # Create the lap object
        pose3d_object = ListenerPose3d("/F1ROS/odom")
        self.lap = Lap(pose3d_object)
        self.map = Map(pose3d_object)

    # Explicit initialization function
    # Class method, so user can call it without instantiation
    @classmethod
    def initGUI(cls, host, console):
        # self.payload = {'image': '', 'shape': []}
        new_instance = cls(host, console)
        return new_instance

    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadImage(self):
        payload = {'image': '', 'shape': ''}

        self.show_lock.acquire()
        show_image = self.show_image
        self.show_lock.release()

        if(show_image == False):
            return payload

    	image = self.hal.getImage()
    	
    	shape = image.shape
        frame = cv2.imencode('.JPEG', image)[1]
        encoded_image = base64.b64encode(frame)
        
        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape

        self.show_lock.acquire()
        self.show_image = False
        self.show_lock.release()
        
        return payload
    
    # Function for student to call
    def showImage(self, image):
    	self.show_lock.acquire()
    	self.show_image = True
    	self.show_lock.release()

    # Function to get the client
    # Called when a new client is received
    def get_client(self, client, server):
        self.client = client
        self.console.set_websocket(self.server, self.client)
        
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
    	payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)
        
        lapped = self.lap.check_threshold()
        self.payload["lap"] = ""
        if(lapped != None):
            self.payload["lap"] = str(lapped)
            
        pos_message = str(self.map.getFormulaCoordinates())
        self.payload["map"] = pos_message
        
        try:
            message = "#gui" + json.dumps(self.payload)
            self.server.send_message(self.client, message)
        except:
            pass
            
    # Function to read the message from websocket
    # Gets called when there is an incoming message from the client
    def get_message(self, client, server, message):
		# Acknowledge Message for GUI Thread
		if(message[:4] == "#ack"):
			self.set_acknowledge(True)
			
		# Message for Console
		elif(message[:4] == "#con"):
			self.console.prompt(message)
    
    # Activate the server
    def run_server(self):
        self.server = WebsocketServer(port=2303, host=self.host)
        self.server.set_fn_new_client(self.get_client)
        self.server.set_fn_message_received(self.get_message)
        self.server.run_forever()

    # Function to reset
    def reset_gui(self):
        self.lap.reset()
        self.map.reset()
        

# This class decouples the user thread
# and the GUI update thread
class ThreadGUI(threading.Thread):
    def __init__(self, gui):
        self.gui = gui
        self.time_cycle = 50
        threading.Thread.__init__(self)
        
    def run(self):
        while(True):
            start_time = datetime.now()
            self.gui.update_gui()
            acknowledge_message = self.gui.get_acknowledge()
            
            while(acknowledge_message == False):
            	acknowledge_message = self.gui.get_acknowledge()
            	
            self.gui.set_acknowledge(False)
            
            finish_time = datetime.now()
            
            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            if(ms < self.time_cycle):
                time.sleep((self.time_cycle-ms) / 1000.0)
