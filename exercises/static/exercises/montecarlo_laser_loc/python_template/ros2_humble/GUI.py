import json
import cv2
import math
import base64
import threading
import time
from datetime import datetime
import websocket
import rclpy
import matplotlib.pyplot as plt

from HAL import getPose3d
from map import Map
from console import start_console

# Graphical User Interface Class

# Matrix colors
red = [0, 0, 255]
orange = [0, 165, 255]
yellow = [0, 255, 255]
green = [0, 255, 0]
blue = [255, 0, 0]
indigo = [130, 0, 75]
violet = [211, 0, 148]


class GUI:
    # Initialization function
    # The actual initialization
    def __init__(self, host):

        # ROS2 init
        if not rclpy.ok():
            rclpy.init(args=None)

        self.payload = {'map': '', 'user': '', 'particles': ''}
        self.server = None
        self.client = None
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)

        self.host = host

        # Particles
        self.particles = []
        # User position
        self.user_position = (0, 0)
        self.user_angle = (0, 0)

        self.ack = False
        self.ack_lock = threading.Lock()

        # Create the lap object
        self.map = Map(getPose3d)

        self.client_thread = threading.Thread(target=self.run_websocket)
        self.client_thread.start()

    # Function to get value of Acknowledge
    def get_acknowledge(self):
        with self.ack_lock:
            ack = self.ack

        return ack

    # Function to get value of Acknowledge
    def set_acknowledge(self, value):
        with self.ack_lock:
            self.ack = value

    def showPosition(self, x, y, angle):
        angle = angle + math.pi
        ay = math.cos(-angle) - math.sin(-angle)
        ax = math.sin(-angle) + math.cos(-angle)
        scale_y = 15; offset_y = 63
        y = scale_y * y + offset_y		
        scale_x = -30; offset_x = 171
        x = scale_x * x + offset_x
        self.user_position = x, y
        self.user_angle = (ax, ay)

    # Function for student to call
    def showParticles(self, particles):
        if len(particles) > 0:
            self.particles = particles
            scale_y = 15; offset_y = 63
            scale_x = -30; offset_x = 171
            for particle in self.particles:                
                particle[1] = scale_y * particle[1] + offset_y                                
                particle[0] = scale_x * particle[0] + offset_x
                # particle[2] = (particle[2]) * 180/math.pi
        else:
            self.particles = []

    # Update the gui
    def update_gui(self):
        # Payload Map Message
        pos_message = self.map.getRobotCoordinates()
        if (pos_message == self.init_coords):
            pos_message = self.start_coords
        ang_message = self.map.getRobotAngle()
        pos_message = str(pos_message + ang_message)
        self.payload["map"] = pos_message

        # Payload User Message
        pos_message_user = self.user_position
        ang_message_user = self.user_angle
        pos_message_user = pos_message_user + ang_message_user
        pos_message_user = str(pos_message_user)
        self.payload["user"] = pos_message_user

        # Payload Particles Message
        if len(self.particles) > 0:
            self.payload["particles"] = json.dumps(self.particles)
        else:
            self.payload["particles"] = json.dumps([])

        message = json.dumps(self.payload)

        if self.client:
            try:
                self.client.send(message)
            except Exception as e:
                print(f"Error sending message: {e}")

    # Function to read the message from websocket
    # Gets called when there is an incoming message from the client
    def on_message(self, message):
        # Acknowledge Message for GUI Thread
        if (message[:4] == "#ack"):
            self.set_acknowledge(True)

    def getMap(self, url):
        return plt.imread(url)
    
    def poseToMap(self, x_prime, y_prime, yaw_prime):
        scale_x = 1024/9.885785
        offset_x = 5.650662
        x = 1181 - (x_prime + offset_x) * scale_x
        scale_y = 1024/9.75819
        offset_y = 4.088577
        y = 24 + (y_prime + offset_y) * scale_y
        yaw = yaw_prime
        return [x, y, yaw]
    
    def mapToPose(self, x, y, yaw):
        scale_x = 1024/9.885785
        offset_x = 5.650662
        x = ((1181 - x) / scale_x) - offset_x
        scale_y = 1024/9.75819
        offset_y = 4.088577
        y = ((-24 + y) / scale_y) - offset_y
        return [x, y, yaw]

    def run_websocket(self):
        while True:
            self.client = websocket.WebSocketApp(self.host, on_message=self.on_message)
            self.client.run_forever(ping_timeout=None, ping_interval=0)

    # Function to reset
    def reset_gui(self):
        self.map.reset()


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

def showPosition(x, y, angle):
    gui_interface.showPosition(x, y, angle)

def showParticles(particles):
    gui_interface.showParticles(particles)

def getMap(url):
    return gui_interface.getMap(url)

def poseToMap(x_prime, y_prime, yaw_prime):
    return gui_interface.poseToMap(x_prime, y_prime, yaw_prime)

def mapToPose(x, y, yaw):
    return gui_interface.mapToPose(x, y, yaw)