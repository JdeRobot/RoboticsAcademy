import json
import cv2
import base64
import threading
import time
from datetime import datetime
from websocket_server import WebsocketServer
import logging

from interfaces.pose3d import ListenerPose3d

from map import Map

# Graphical User Interface Class
class GUI:
    # Initialization function
    # The actual initialization
    def __init__(self, host, console):
        t = threading.Thread(target=self.run_server)
        
        self.payload = {'canvas': None,'image': '', 'shape': []}
        self.server = None
        self.client = None
        
        self.host = host

        self.payload_lock = threading.Lock()
        
        # Take the console object to set the same websocket and client
        self.console = console
        t.start()
        
        # Create the lap object
        pose3d_object = ListenerPose3d("/roombaROS/odom")
        self.map = Map(pose3d_object)

    # Explicit initialization function
    # Class method, so user can call it without instantiation
    @classmethod
    def initGUI(cls, host, console):
        # self.payload = {'image': '', 'shape': []}
        new_instance = cls(host, console)
        return new_instance

    # Function for student to call
    # Encodes the image as a JSON string and sends through the WS
    def showImage(self, image):
        shape = image.shape
        frame = cv2.imencode('.JPEG', image)[1]
        encoded_image = base64.b64encode(frame)
        
        self.payload_lock.acquire()
        self.payload['image'] = encoded_image.decode('utf-8')
        self.payload['shape'] = shape
        self.payload_lock.release()

    # Function to get the client
    # Called when a new client is received
    def get_client(self, client, server):
        self.client = client
        self.console.set_websocket(self.server, self.client)
        
    # Update the gui
    def update_gui(self):
        pos_message = self.map.getRobotCoordinates()
        ang_message = self.map.getRobotAngle()
        pos_message = str(pos_message + ang_message)
        pos_message = "#map" + pos_message
        
        try:
            #self.server.send_message(self.client, message)
            self.server.send_message(self.client, pos_message)
        except:
            pass
    
    # Activate the server
    def run_server(self):
        self.server = WebsocketServer(port=2303, host=self.host)
        self.server.set_fn_new_client(self.get_client)
        self.server.set_fn_message_received(self.console.prompt)
        self.server.run_forever()

    # Function to reset
    def reset_gui(self):
        self.map.reset()
        

# This class decouples the user thread
# and the GUI update thread
class ThreadGUI(threading.Thread):
    def __init__(self, gui):
        self.gui = gui
        self.time_cycle = 200
        threading.Thread.__init__(self)
        
    def run(self):
        while(True):
            start_time = datetime.now()
            self.gui.update_gui()
            
            finish_time = datetime.now()
            
            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            if(ms < self.time_cycle):
                time.sleep((self.time_cycle-ms) / 1000.0)
