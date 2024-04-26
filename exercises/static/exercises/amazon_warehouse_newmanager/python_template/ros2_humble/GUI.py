import json
import re
import threading
import time
from datetime import datetime
import websocket
import rclpy
from console import start_console
import matplotlib.pyplot as plt

from map import Map
from interfaces.pose3d import ListenerPose3d

# Graphical User Interface Class
class GUI:
    # Initialization function
    # The actual initialization
    def __init__(self, host):

        # ROS2 init
        rclpy.init(args=None)
        node = rclpy.create_node('GUI')

        self.payload = {'map': '', 'array': ''}

        # GUI websocket
        self.host = host
        self.client = None

        # For path
        self.array_lock = threading.Lock()
        self.array = None

        self.ack = False
        self.ack_lock = threading.Lock()

        self.pose3d_object = ListenerPose3d("/amazon_robot/odom")

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.pose3d_object)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # Create the map object    
        self.map = Map(self.pose3d_object)
        self.client_thread = threading.Thread(target=self.run_websocket)
        self.client_thread.start()

    def run_websocket(self):
        while True:
            self.client = websocket.WebSocketApp(self.host, on_message=self.on_message)
            self.client.run_forever(ping_timeout=None, ping_interval=0)

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
        # Payload path array
        self.payload["array"] = self.array

        # Payload Map Message
        pos_message = self.map.getRobotCoordinates()
        ang_message = self.map.getRobotAngle()
        pos_message = str(pos_message + ang_message)
        self.payload["map"] = pos_message

        message = json.dumps(self.payload)
        if self.client:
            try:
                self.client.send(message)
            except Exception as e:
                print(f"Error sending message: {e}")

    def on_message(self, ws, message):
        """Handles incoming messages from the websocket client."""
        if message.startswith("#ack"):
            self.set_acknowledge(True)

    # Function to reset
    def reset_gui(self):
        self.map.reset()

#------------------------------------------------------------#    
    # Process the array(ideal path) to be sent to websocket
    def showPath(self, array):
        array_scaled = []
        for wp in array:
            array_scaled.append([wp[0] * 0.72, wp[1] * 0.545])

        print("Path array: " + str(array_scaled))
        self.array_lock.acquire()

        strArray = ''.join(str(e) for e in array_scaled)
        print("strArray: " + str(strArray))

        # Remove unnecesary spaces in the array to avoid JSON syntax error in javascript
        strArray = re.sub(r"\[[ ]+", "[", strArray)
        strArray = re.sub(r"[ ]+", ", ", strArray)
        strArray = re.sub(r",[ ]+]", "]", strArray)
        strArray = re.sub(r",,", ",", strArray)
        strArray = re.sub(r"]\[", "],[", strArray)
        strArray = "[" + strArray + "]"
        print("strArray2: " + str(strArray))

        self.array = strArray
        self.array_lock.release()
    
    def getMap(self, url):
        return plt.imread(url)
#------------------------------------------------------------#


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

def showPath(array):
    return gui_interface.showPath(array)

def getMap(url):
    return gui_interface.getMap(url)