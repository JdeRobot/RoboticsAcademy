import re
import matplotlib.pyplot as plt
import json
import threading
import time
import websocket
from src.manager.ram_logging.log_manager import LogManager
from gazebo_msgs.srv import SetEntityState, GetEntityState
import rclpy
from console import start_console
from map import Map
from HAL import getPose3d


class ThreadingGUI:

    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):

        # ROS 2 init
        if not rclpy.ok():
            rclpy.init()

        # Execution control vars
        self.out_period = 1.0 / freq

        self.ack = True
        self.ack_lock = threading.Lock()

        self.array_lock = threading.Lock()
        self.array = None
        
        self.running = True
        
        self.host = host
        self.node = rclpy.create_node("node")

        # Payload vars
        self.payload = {'map': '', 'array': ''}
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)
        self.map = Map(getPose3d)

        # Initialize and start the WebSocket client thread
        threading.Thread(target=self.run_websocket, daemon=True).start()

        # Initialize and start the image sending thread (GUI out thread)
        threading.Thread(
            target=self.gui_out_thread, name="gui_out_thread", daemon=True
        ).start()

    # Init websocket client
    def run_websocket(self):
        self.client = websocket.WebSocketApp(self.host, on_message=self.gui_in_thread)
        self.client.run_forever(ping_timeout=None, ping_interval=0)

    # Process incoming messages to the GUI
    def gui_in_thread(self, ws, message):

        # In this case, incoming msgs can only be acks
        if "ack" in message:
            with self.ack_lock:
                self.ack = True

    # Process outcoming messages from the GUI
    def gui_out_thread(self):
        while self.running:
            start_time = time.time()

            # Check if a new map should be sent
            self.send_map()
            # with self.ack_lock:
            #     if self.ack and self.map is not None:
            #         self.ack = False

            # Maintain desired frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, self.out_period - elapsed)
            time.sleep(sleep_time)

    # Prepares and sends a map to the websocket server
    def send_map(self):
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


host = "ws://127.0.0.1:2303"
gui = ThreadingGUI(host)

# Redirect the console
start_console()

def showPath(array):
    return gui.showPath(array)

def getMap(url):
    return gui.getMap(url)