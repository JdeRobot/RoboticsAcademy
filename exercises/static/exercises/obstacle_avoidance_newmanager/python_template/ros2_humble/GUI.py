import json
import cv2
import base64
import threading
import time
from datetime import datetime
import websocket
import logging
import rclpy

from HAL import getLaserData

from lap import Lap
from map import Map

from console import start_console

# Graphical User Interface Class
class GUI:
    """Graphical User Interface class"""

    def __init__(self, host):
        """Initializes the GUI"""

        print("GUI IS BEING INITIALIZED\n\n\n\n")

        # ROS2 init
        if not rclpy.ok():
            rclpy.init(args=None)
        node = rclpy.create_node('GUI')

        self.payload = {'lap': '', 'map': ''}

        self.server = None
        self.client = None
        self.host = host
        
        self.ack = False
        self.ack_lock = threading.Lock()

        # Create the map and lap objects
        self.map = Map(getLaserData)
        self.lap = Lap(self.map)

        self.client_thread = threading.Thread(target=self.run_websocket)
        self.client_thread.start()


    def run_websocket(self):
        while True:
            self.client = websocket.WebSocketApp(self.host, on_message=self.on_message)
            self.client.run_forever(ping_timeout=None, ping_interval=0)

    def showForces(self, vec1, vec2, vec3):
        """Function for student to call"""
        self.map.setCar(vec1[0], vec1[1])
        self.map.setObs(vec2[0], vec2[1])
        self.map.setAvg(vec3[0], vec3[1])

    def showLocalTarget(self, newVec):
        """Function for student to call"""
        self.map.setTargetPos(newVec[0], newVec[1])
        
    def update_gui(self):
        """Updates the GUI with the latest map information."""

        # Payload Lap Message
        lapped = self.lap.check_threshold()
        lap_message = ""
        if(lapped != None):
            self.payload["lap"] = str(lapped)

        # Payload Map Message
        map_message = self.map.get_json_data()
        self.payload["map"] = map_message

        message = json.dumps(self.payload)
        if self.client:
            try:
                self.client.send(message)
            except Exception as e:
                print(f"Error sending message: {e}")

    def on_message(self, ws, message):
        """Handles incoming messages from the websocket client."""
        if (message[:4] == "#ack"):
            with self.ack_lock:
                self.ack = True

    def reset_gui(self):
        """Resets the GUI to its initial state."""
        self.map.reset()
        self.lap.reset()


class ThreadGUI:
    """Class to manage GUI updates and frequency measurements in separate threads."""

    def __init__(self, gui):
        """Initializes the ThreadGUI with a reference to the GUI instance."""
        self.gui = gui
        self.iteration_counter = 0
        self.running = True

    def start(self):
        """Starts the GUI, frequency measurement, and real-time factor threads."""
        self.gui_thread = threading.Thread(target=self.run)
        self.gui_thread.start()
        print("GUI Thread Started!")

    def run(self):
        """Main loop to update the GUI at regular intervals."""
        while self.running:
            start_time = datetime.now()
            self.gui.update_gui()
            self.iteration_counter += 1
            finish_time = datetime.now()
            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            sleep_time = max(0, (50 - ms) / 1000.0)
            time.sleep(sleep_time)


# Create a GUI interface
host = "ws://127.0.0.1:2303"
gui_interface = GUI(host)

# Spin a thread to keep the interface updated
thread_gui = ThreadGUI(gui_interface)
thread_gui.start()

start_console()

def showImage(image):
    gui_interface.showImage(image)

def showForces(vec1, vec2, vec3):
    gui_interface.showForces(vec1, vec2, vec3)

def showLocalTarget(newVec):
    return gui_interface.showLocalTarget(newVec)

#TODO: change this to another file, not GUI

def getNextTarget():
    return gui_interface.map.getNextTarget()

def setTargetx(x):
    gui_interface.map.targetx = x

def setTargety(y):
    gui_interface.map.targety = y
