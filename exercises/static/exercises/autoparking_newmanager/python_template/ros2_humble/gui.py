import json
import cv2
import base64
import threading
import time
from datetime import datetime
import websocket
import subprocess
import logging
import os
import rclpy
from interfaces.pose3d import ListenerPose3d

from map import Map
from interfaces.laser import ListenerLaser

# Graphical User Interface Class


class GUI:
    """Graphical User Interface class"""

    def __init__(self, host, hal):
        print("GUI IS BEING CALLED\n\n\n\n")
        """Initializes the GUI"""
        self.payload = {'map': ''}
        self.server = None
        self.client = None
        self.host = host
        self.acknowledge = False
        self.acknowledge_lock = threading.Lock()
        self.hal = hal
        # Create Sensor objects
        laser_object_f = ListenerLaser("/prius_autoparking/scan_front")
        laser_object_r = ListenerLaser("/prius_autoparking/scan_side")
        laser_object_b = ListenerLaser("/prius_autoparking/scan_back")
        pose3d_object = ListenerPose3d("/prius_autoparking/odom")

        # Spin nodes so that subscription callbacks load topic data
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(pose3d_object)
        executor.add_node(laser_object_f)
        executor.add_node(laser_object_r)
        executor.add_node(laser_object_b)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # create Map object
        self.map = Map(laser_object_f, laser_object_r, laser_object_b, pose3d_object)

        self.client_thread = threading.Thread(target=self.run_websocket)
        self.client_thread.start()

    def on_error(self, error):
        f = open("/ws_gui_error.log", "a")
        f.write(str(error))
        f.close()
    
    def on_close(wsapp, close_status_code, close_msg):
        f = open("/ws_gui_close.log", "a")
        f.write(str(close_status_code))
        f.write("\n")
        f.write(str(close_msg))
        f.close()

    def run_websocket(self):
        f = open("/ws_gui_open.log", "a")
        f.write("WS GUI OPENED")
        f.close()
        self.server = websocket.WebSocketApp('ws://127.0.0.1:2303',
                                                on_message=self.on_message,on_error=self.on_error, on_close=self.on_close)
        self.server.run_forever(ping_timeout=None, ping_interval=0)

    @classmethod
    def initGUI(cls):
        """Initializes the GUI class."""
        pass

    def on_open(self, ws):
        """Handles new websocket client connections."""
        print('connected')

    def get_acknowledge(self):
        """Gets the acknowledge status."""
        self.acknowledge_lock.acquire()
        acknowledge = self.acknowledge
        self.acknowledge_lock.release()
        return acknowledge

    def set_acknowledge(self, value):
        """Sets the acknowledge status."""
        self.acknowledge_lock.acquire()
        self.acknowledge = value
        self.acknowledge_lock.release()

    def update_gui(self):
        """Updates the GUI with the latest map information."""
        map_message = self.map.get_json_data()
        self.payload["map"] = map_message
        message = json.dumps(self.payload)
        if self.server:
            try:
                self.server.send(message)
            except Exception as e:
                print(f"Error sending message: {e}")

    def on_message(self, ws, message):
        """Handles incoming messages from the websocket client."""
        if message.startswith("#ack"):
            self.set_acknowledge(True)

    def reset_gui(self):
        """Resets the GUI to its initial state."""
        self.map.reset()


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
            if self.gui.server:
                try:
                    self.gui.server.send(message)
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

