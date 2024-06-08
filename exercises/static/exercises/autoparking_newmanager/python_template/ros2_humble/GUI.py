import json
import threading
import time
from datetime import datetime
import websocket
import logging
import rclpy

from HAL import getFrontLaserData, getRightLaserData, getBackLaserData
from map import Map
from console import start_console

# Graphical User Interface Class
class GUI:
    """Graphical User Interface class"""

    def __init__(self, host):
        print("GUI IS BEING CALLED\n\n\n\n")
        """Initializes the GUI"""

        # ROS2 init
        if not rclpy.ok():
            rclpy.init(args=None)

        self.payload = {'map': ''}
        self.client = None
        self.host = host
        self.ack = False
        self.ack_lock = threading.Lock()

        # create Map object
        self.map = Map(getFrontLaserData, getRightLaserData, getBackLaserData)

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