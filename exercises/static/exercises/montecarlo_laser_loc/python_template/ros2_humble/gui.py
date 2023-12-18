import json
import cv2
import numpy as np
import math
import base64
import threading
import time
from datetime import datetime
from websocket_server import WebsocketServer
import logging
import os
import rclpy
import matplotlib.pyplot as plt
from shared.image import SharedImage
from interfaces.pose3d import ListenerPose3d
import matplotlib.pyplot as plt

from map import Map

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
    def __init__(self, host, hal):
        t = threading.Thread(target=self.run_server)

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

        self.acknowledge = False
        self.acknowledge_lock = threading.Lock()

        self.shared_image = SharedImage("guiimage")

        self.hal = hal
        t.start()

        # Create the lap object
        self.map = Map(self.hal.pose3d)

    # Explicit initialization function
    # Class method, so user can call it without instantiation
    @classmethod
    def initGUI(cls, host, console):
        # self.payload = {'map': ''}
        new_instance = cls(host, console)
        return new_instance

    # Function to get the client
    # Called when a new client is received
    def get_client(self, client, server):
        self.client = client

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

    # encode the image data to be sent to websocket
    def payloadImage(self):

        image = self.shared_image.get()
        payload = {'image': '', 'shape': ''}

        shape = image.shape
        frame = cv2.imencode('.PNG', image)[1]
        encoded_image = base64.b64encode(frame)

        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape

        return payload

    def showPosition(self, x, y, angle):
        angle = angle
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
                particle[2] = (particle[2] + math.pi) * 180/math.pi
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

        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        message = "#gui" + json.dumps(self.payload)
        self.server.send_message(self.client, message)

    # Function to read the message from websocket
    # Gets called when there is an incoming message from the client
    def get_message(self, client, server, message):
        # Acknowledge Message for GUI Thread
        if (message[:4] == "#ack"):
            self.set_acknowledge(True)

    def getMap(self, url):
        return plt.imread(url)
    
    def poseToMap(self, x_prime, y_prime, yaw_prime):
        scale_x = 1024/9.885785
        offset_x = 5.650662
        x = 1169 - (x_prime + offset_x) * scale_x
        scale_y = 1024/9.75819
        offset_y = 4.088577
        y = (y_prime + offset_y) * scale_y
        yaw = -yaw_prime
        return [x, y, yaw]

    
    def mapToPose(self, x, y, yaw):
        scale_x = 1024/9.885785
        offset_x = 5.650662
        x = ((1169 - x) / scale_x) - offset_x
        scale_y = 1024/9.75819
        offset_y = 4.088577
        y = (y / scale_y) - offset_y
        return [x, y, -yaw]

    # Activate the server
    def run_server(self):
        self.server = WebsocketServer(port=2303, host=self.host)
        self.server.set_fn_new_client(self.get_client)
        self.server.set_fn_message_received(self.get_message)

        home_dir = os.path.expanduser('~')

        logged = False
        while not logged:
            try:
                f = open(f"{home_dir}/ws_gui.log", "w")
                f.write("websocket_gui=ready")
                f.close()
                logged = True
            except:
                time.sleep(0.1)

        self.server.run_forever()

    # Function to reset
    def reset_gui(self):
        self.map.reset()


# This class decouples the user thread
# and the GUI update thread
class ThreadGUI:
    def __init__(self, gui):
        self.gui = gui

        # Time variables
        self.ideal_cycle = 80
        self.measured_cycle = 80
        self.iteration_counter = 0

    # Function to start the execution of threads
    def start(self):
        self.measure_thread = threading.Thread(target=self.measure_thread)
        self.thread = threading.Thread(target=self.run)

        self.measure_thread.start()
        self.thread.start()

        print("GUI Thread Started!")

    # The measuring thread to measure frequency
    def measure_thread(self):
        while (self.gui.client == None):
            pass

        previous_time = datetime.now()
        while (True):
            # Sleep for 2 seconds
            time.sleep(2)

            # Measure the current time and subtract from previous time to get real time interval
            current_time = datetime.now()
            dt = current_time - previous_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * \
                1000 + dt.microseconds / 1000.0
            previous_time = current_time

            # Get the time period
            try:
                # Division by zero
                self.measured_cycle = ms / self.iteration_counter
            except:
                self.measured_cycle = 0

            # Reset the counter
            self.iteration_counter = 0

    # The main thread of execution
    def run(self):
        while (self.gui.client == None):
            pass

        while (True):
            start_time = datetime.now()
            self.gui.update_gui()
            acknowledge_message = self.gui.get_acknowledge()

            while (acknowledge_message == False):
                acknowledge_message = self.gui.get_acknowledge()

            self.gui.set_acknowledge(False)

            finish_time = datetime.now()
            self.iteration_counter = self.iteration_counter + 1

            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * \
                1000 + dt.microseconds / 1000.0
            if (ms < self.ideal_cycle):
                time.sleep((self.ideal_cycle-ms) / 1000.0)
