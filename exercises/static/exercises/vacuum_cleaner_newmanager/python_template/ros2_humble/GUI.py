import json
import threading
import time
from datetime import datetime
import websocket
import rclpy
from HAL import getPose3d
from console import start_console
from map import Map

# Graphical User Interface Class
class GUI:
    # Initialization function
    # The actual initialization
    def __init__(self, host):

        # ROS2 init
        if not rclpy.ok():
            rclpy.init(args=None)

        self.payload = {'map': '', 'user': ''}
        self.server = None
        self.client = None
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)

        self.host = host

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

    # Update the gui
    def update_gui(self):
        # Payload Map Message
        pos_message = self.map.getRobotCoordinates()
        if (pos_message == self.init_coords):
            pos_message = self.start_coords
        ang_message = self.map.getRobotAngle()
        pos_message = str(pos_message + ang_message)
        self.payload["map"] = pos_message

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

# One class GUI

# import json
# import cv2
# import base64
# import threading
# import time
# import websocket
# from src.manager.ram_logging.log_manager import LogManager
# from gazebo_msgs.srv import SetEntityState, GetEntityState
# import rclpy
# from console import start_console
# from map import Map
# from HAL import getPose3d


# class ThreadingGUI:

#     def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):

#         # ROS 2 init
#         if not rclpy.ok():
#             rclpy.init()

#         # Execution control vars
#         self.out_period = 1.0 / freq
#         self.ack = True
#         self.ack_lock = threading.Lock()
#         self.running = True
#         self.host = host
#         self.node = rclpy.create_node("node")

#         # Payload vars
#         self.msg = {"pos_msg": "", "ang_msg": ""}
#         self.init_coords = (171, 63)
#         self.start_coords = (201, 85.5)
#         self.map = Map(getPose3d)

#         # Initialize and start the WebSocket client thread
#         threading.Thread(target=self.run_websocket, daemon=True).start()

#         # Initialize and start the image sending thread (GUI out thread)
#         threading.Thread(
#             target=self.gui_out_thread, name="gui_out_thread", daemon=True
#         ).start()

#     # Init websocket client
#     def run_websocket(self):
#         self.client = websocket.WebSocketApp(self.host, on_message=self.gui_in_thread)
#         self.client.run_forever(ping_timeout=None, ping_interval=0)

#     # Process incoming messages to the GUI
#     def gui_in_thread(self, ws, message):

#         # In this case, incoming msgs can only be acks
#         if "ack" in message:
#             with self.ack_lock:
#                 self.ack = True

#     # Process outcoming messages from the GUI
#     def gui_out_thread(self):
#         while self.running:
#             start_time = time.time()

#             # Check if a new map should be sent
#             with self.ack_lock:
#                 if self.ack and self.map is not None:
#                     self.send_map()
#                     self.ack = False

#             # Maintain desired frequency
#             elapsed = time.time() - start_time
#             sleep_time = max(0, self.out_period - elapsed)
#             time.sleep(sleep_time)

#     # Prepares and sends a map to the websocket server
#     def send_map(self):

#         # Get the necessary info
#         pos_message = self.map.getRobotCoordinates()
#         if pos_message == self.init_coords:
#             pos_message = self.start_coords
#         ang_message = self.map.getRobotAngle()

#         self.msg["pos_msg"] = pos_message
#         self.msg["ang_msg"] = ang_message
#         message = json.dumps(self.msg)
#         try:
#             if self.client:
#                 self.client.send(message)
#         except Exception as e:
#             LogManager.logger.info(f"Error sending message: {e}")


# host = "ws://127.0.0.1:2303"
# gui = ThreadingGUI(host)

# # Redirect the console
# start_console()
