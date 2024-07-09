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

# Matrix colors
red = [0, 0, 255]
orange = [0, 165, 255]
yellow = [0, 255, 255]
green = [0, 255, 0]
blue = [255, 0, 0]
indigo = [130, 0, 75]
violet = [211, 0, 148]


class GUI:
    """Graphical User Interface class"""

    def __init__(self, host):
        """Initializes the GUI"""

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
        self.running = True
        self.iteration_counter = 0

        # Create the map object
        self.map = Map(getPose3d)

        # Start the websocket and GUI update threads
        self.websocket_thread = threading.Thread(target=self.run_websocket)
        self.update_thread = threading.Thread(target=self.run)
        self.websocket_thread.start()
        self.update_thread.start()

    def run_websocket(self):
        while self.running:
            self.client = websocket.WebSocketApp(self.host, on_message=self.on_message)
            self.client.run_forever(ping_timeout=None, ping_interval=0)

    def update_gui(self):
        """Updates the GUI with the latest map information."""
        # Payload Map Message
        pos_message = self.map.getRobotCoordinates()
        if pos_message == self.init_coords:
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
        if self.particles:
            self.payload["particles"] = json.dumps(self.particles)
        else:
            self.payload["particles"] = json.dumps([])

        message = json.dumps(self.payload)
        if self.client:
            try:
                self.client.send(message)
            except Exception as e:
                print(f"Error sending message: {e}")

    def on_message(self, ws, message):
        """Handles incoming messages from the websocket client."""
        if message.startswith("#ack"):
            with self.ack_lock:
                self.ack = True

    def get_acknowledge(self):
        with self.ack_lock:
            return self.ack

    def set_acknowledge(self, value):
        with self.ack_lock:
            self.ack = value

    def showPosition(self, x, y, angle):
        angle += math.pi
        ay = math.cos(-angle) - math.sin(-angle)
        ax = math.sin(-angle) + math.cos(-angle)
        scale_y = 15
        offset_y = 63
        y = scale_y * y + offset_y
        scale_x = -30
        offset_x = 171
        x = scale_x * x + offset_x
        self.user_position = x, y
        self.user_angle = (ax, ay)

    def showParticles(self, particles):
        if particles:
            self.particles = particles
            scale_y = 15
            offset_y = 63
            scale_x = -30
            offset_x = 171
            for particle in self.particles:
                particle[1] = scale_y * particle[1] + offset_y
                particle[0] = scale_x * particle[0] + offset_x
        else:
            self.particles = []

    def getMap(self, url):
        return plt.imread(url)

    def poseToMap(self, x_prime, y_prime, yaw_prime):
        scale_x = 1024 / 9.885785
        offset_x = 5.650662
        x = 1181 - (x_prime + offset_x) * scale_x
        scale_y = 1024 / 9.75819
        offset_y = 4.088577
        y = 24 + (y_prime + offset_y) * scale_y
        yaw = yaw_prime
        return [x, y, yaw]

    def mapToPose(self, x, y, yaw):
        scale_x = 1024 / 9.885785
        offset_x = 5.650662
        x = ((1181 - x) / scale_x) - offset_x
        scale_y = 1024 / 9.75819
        offset_y = 4.088577
        y = ((-24 + y) / scale_y) - offset_y
        return [x, y, yaw]

    def run(self):
        """Main loop to update the GUI at regular intervals."""
        while self.running:
            start_time = datetime.now()
            self.update_gui()
            self.iteration_counter += 1
            finish_time = datetime.now()
            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            sleep_time = max(0, (50 - ms) / 1000.0)
            time.sleep(sleep_time)


# Create a GUI interface
host = "ws://127.0.0.1:2303"
gui_interface = GUI(host)

# Start the console
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
