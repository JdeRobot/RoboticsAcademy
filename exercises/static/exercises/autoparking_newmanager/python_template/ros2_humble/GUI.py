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
        self.running = True
        self.iteration_counter = 0

        # Create Map object
        self.map = Map(getFrontLaserData, getRightLaserData, getBackLaserData)

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
                self.ack = True

    def reset_gui(self):
        """Resets the GUI to its initial state."""
        self.map.reset()

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
