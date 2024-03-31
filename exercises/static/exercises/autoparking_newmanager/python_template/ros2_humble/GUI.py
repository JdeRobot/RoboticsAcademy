import json
import threading
import time
from datetime import datetime
import websocket
import logging
import rclpy
from interfaces.pose3d import ListenerPose3d

from map import Map
from interfaces.laser import ListenerLaser

from console import start_console
# Graphical User Interface Class
class GUI:
    """Graphical User Interface class"""

    def __init__(self, host):
        print("GUI IS BEING CALLED\n\n\n\n")
        """Initializes the GUI"""

        # ROS2 init
        rclpy.init(args=None)
        node = rclpy.create_node('GUI')

        self.payload = {'map': ''}
        self.client = None
        self.host = host
        self.ack = False
        self.ack_lock = threading.Lock()

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

        threading.Thread(target=self.run_websocket).start()

    def run_websocket(self):
        self.client = websocket.WebSocketApp(self.host, on_message=self.on_message)
        self.client.run_forever(ping_timeout=None, ping_interval=0)

    def update_gui(self):
        """Updates the GUI with the latest map information."""
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
        if message.startswith("#ack"):
            with self.ack_lock:
                self.ack=True

    def reset_gui(self):
        """Resets the GUI to its initial state."""
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