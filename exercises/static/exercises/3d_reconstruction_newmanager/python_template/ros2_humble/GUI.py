import json
import cv2
import base64
import threading
import time
from datetime import datetime
import subprocess
import websocket
import os
import numpy as np

from console import start_console


# Graphical User Interface Class
class GUI:
    # Initialization function
    # The actual initialization
    def __init__(self, host):
        self.payload = {'img1': '', 'img2': '', 'pts': '', 'match': '', 'p_match': 'F'}

        # ROS2 init
        rclpy.init(args=None)
        node = rclpy.create_node('GUI')

        self.server = None
        self.client = None

        self.host = host

        # Image variables
        self.image1_to_be_shown = None
        self.image2_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.ack = False
        self.ack_lock = threading.Lock()

        self.point_to_save = []
        self.point_to_send = []

        self.matching_to_save = []
        self.duplicate_matching = False
        self.matching_to_send = []
        self.paint_matching = "F"

        self.client_thread = threading.Thread(target=self.run_websocket)
        self.client_thread.start()

    def run_websocket(self):
        while True:
            self.client = websocket.WebSocketApp(self.host, on_message=self.on_message,)
            self.client.run_forever(ping_timeout=None, ping_interval=0)


    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image1_to_be_shown = self.image1_to_be_shown
            image2_to_be_shown = self.image2_to_be_shown

        image1 = image1_to_be_shown
        payload1 = {'img': ''}
        image2 = image2_to_be_shown
        payload2 = {'img': ''}

        if(image_to_be_shown_updated == False):
            return payload1, payload2

        image1 = cv2.resize(image1, (0, 0), fx=0.50, fy=0.50)
        frame1 = cv2.imencode('.JPEG', image1)[1]
        encoded_image1 = base64.b64encode(frame1)
        payload1['img'] = encoded_image1.decode('utf-8')

        image2 = cv2.resize(image2, (0, 0), fx=0.50, fy=0.50)
        frame2 = cv2.imencode('.JPEG', image2)[1]
        encoded_image2 = base64.b64encode(frame2)
        payload2['img'] = encoded_image2.decode('utf-8')

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload1, payload2

    # Function for student to call
    def showImages(self, image1, image2, paint_matching):
        self.paint_matching = paint_matching
        if paint_matching:
            self.paint_matching = "T"
        else:
            self.paint_matching = "F"
        if (np.all(self.image1_to_be_shown == image1) == False or np.all(self.image2_to_be_shown == image2) == False):
            with self.image_show_lock:
                self.image1_to_be_shown = image1
                self.image2_to_be_shown = image2
                self.image_to_be_shown_updated = True

    # Function to get value of Acknowledge
    def get_acknowledge(self):
        with self.ack:
            ack = self.acknowledge

        return ack

    # Function to get value of Acknowledge
    def set_acknowledge(self, value):
        with self.ack_lock:
            self.ack = value

    # Update the gui
    def update_gui(self):
        payload1, payload2 = self.payloadImage()
        self.payload["img1"] = json.dumps(payload1)
        self.payload["img2"] = json.dumps(payload2)
        length_point_send = len(self.point_to_send)

        if (length_point_send != 0):
            if (length_point_send > 100):
                self.payload["pts"] = json.dumps(self.point_to_send[0:100])
                del self.point_to_send[0:100]
            else:
                self.payload["pts"] = json.dumps(self.point_to_send)
                del self.point_to_send[0:length_point_send]
        else:
            self.payload["pts"] = json.dumps([])

        length_matching_send = len(self.matching_to_send)
        self.payload["match"] = json.dumps(self.matching_to_send)
        del self.matching_to_send[0:length_matching_send]

        self.payload["p_match"] = self.paint_matching


        # Payload Point Message
        message = json.dumps(self.payload)
        if self.client:
            try:
                self.client.send(message)
            except Exception as e:
                print(f"Error sending message: {e}")

    # Show new points
    def ShowNewPoints(self, points):
        duplicate_point = False
        for i in range(0, len(points)):
            for j in range(0, len(self.point_to_save)):
                if (self.point_to_save[j] == points[i]):
                    duplicate_point = True
            if (duplicate_point == False):
                self.point_to_save.append(points[i])
                self.point_to_send.append(points[i])
            else:
                duplicate_point = False

    # Show all points
    def ShowAllPoints(self, points):
        number_equal_points = 0
        for i in range(0, len(points)):
            for j in range(0, len(self.point_to_save)):
                if (self.point_to_save[j] == points[i]):
                    number_equal_points += 1

        if (number_equal_points != len(points)):
            self.ClearAllPoints()
            for i in range(0, len(points)):
                self.point_to_save.append(points[i])
                self.point_to_send.append(points[i])

    # Show image matching
    def showImageMatching(self, x1, y1, x2, y2):
        matching = [x1, y1, x2, y2]

        for i in range(0, len(self.matching_to_save)):
            if ((self.matching_to_save[i] == matching) == True):
                self.duplicate_matching = True

        if (self.duplicate_matching == False):
            self.matching_to_save.append(matching)
            self.matching_to_send.append(matching)
        else:
            self.duplicate_matching = False

    # Function to reset
    def ClearAllPoints(self):
        self.point_to_save = []
        self.point_to_send = []
        self.matching_to_save = []
        self.matching_to_send = []
        self.server.send_message(self.client, "#res")

    def on_message(self, ws, message):
        """Handles incoming messages from the websocket client."""
        if message.startswith("#ack"):
            self.set_acknowledge(True)


    def reset_gui(self):
        self.ClearAllPoints()


class ThreadGUI:
    """Class to manage GUI updates and frequency measurements in separate threads."""

    def __init__(self, gui):
        """Initializes the ThreadGUI with a reference to the GUI instance."""
        self.gui = gui
        self.ideal_cycle = 80
        self.real_time_factor = 0
        self.frequency_message = {'brain': '', 'gui': '', 'rtf': ''}
        self.iteration_counter = 0
        self.running = True

    def start(self):
        """Starts the GUI, frequency measurement, and real-time factor threads."""
        self.frequency_thread = threading.Thread(target=self.measure_and_send_frequency)
        self.gui_thread = threading.Thread(target=self.run)
        self.rtf_thread = threading.Thread(target=self.get_real_time_factor)
        self.frequency_thread.start()
        self.gui_thread.start()
        self.rtf_thread.start()
        print("GUI Thread Started!")

    def get_real_time_factor(self):
        """Continuously calculates the real-time factor."""
        while True:
            time.sleep(2)
            args = ["gz", "stats", "-p"]
            stats_process = subprocess.Popen(args, stdout=subprocess.PIPE)
            with stats_process.stdout:
                for line in iter(stats_process.stdout.readline, b''):
                    stats_list = [x.strip() for x in line.split(b',')]
                    self.real_time_factor = stats_list[0].decode("utf-8")

    def measure_and_send_frequency(self):
        """Measures and sends the frequency of GUI updates and brain cycles."""
        previous_time = datetime.now()
        while self.running:
            time.sleep(2)
            current_time = datetime.now()
            dt = current_time - previous_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            previous_time = current_time
            measured_cycle = ms / self.iteration_counter if self.iteration_counter > 0 else 0
            self.iteration_counter = 0
            brain_frequency = round(1000 / measured_cycle, 1) if measured_cycle != 0 else 0
            gui_frequency = round(1000 / self.ideal_cycle, 1)
            self.frequency_message = {'brain': brain_frequency, 'gui': gui_frequency, 'rtf': self.real_time_factor}
            message = json.dumps(self.frequency_message)
            if self.gui.client:
                try:
                    self.gui.client.send(message)
                except Exception as e:
                    print(f"Error sending frequency message: {e}")

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

def showImages(image1, image2, paint_matching):
    gui_interface.showImages(image1, image2, paint_matching)

def ShowNewPoints(points):
    gui_interface.ShowNewPoints(points)

def ShowAllPoints(points):
    gui_interface.ShowAllPoints(points)

def showImageMatching(x1, y1, x2, y2):
    gui_interface.showImageMatching(x1, y1, x2, y2)

def ClearAllPoints():
    gui_interface.ClearAllPoints()
