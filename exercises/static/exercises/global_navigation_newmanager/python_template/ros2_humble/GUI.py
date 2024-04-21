import json
import cv2
import base64
import threading
import time
from datetime import datetime
import websocket
import logging
import rclpy
from interfaces.pose3d import ListenerPose3d
import numpy as np
from shared.image import SharedImage
import re

from map import Map
from interfaces.laser import ListenerLaser

from console import start_console
# Graphical User Interface Class
class GUI:
    """Graphical User Interface class"""

    def __init__(self, host, hal):
        """Initializes the GUI"""

        print("GUI IS BEING INITIALIZED\n\n\n\n")

        # ROS2 init
        rclpy.init(args=None)
        node = rclpy.create_node('GUI')

        self.payload = {'image': '', 'map': '', 'array': ''}
        self.server = None
        self.client = None
        self.host = host
        self.ack = False
        self.ack_lock = threading.Lock()
        self.array = None
        self.array_lock = threading.Lock()
        self.hal = hal #TODO: check if necessary

        self.shared_image = SharedImage("numpyimage")

        # Create Sensor objects
        pose3d_object = ListenerPose3d("/taxi_holo/odom")

        # Spin nodes so that subscription callbacks load topic data
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(pose3d_object)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # create Map object
        self.map = Map(pose3d_object)

        threading.Thread(target=self.run_websocket).start()

    def run_websocket(self):
        self.client = websocket.WebSocketApp(self.host, on_message=self.on_message)
        self.client.run_forever(ping_timeout=None, ping_interval=0)

    def payloadImage(self):
        """Encodes the image data to be sent to websocket"""
        image = self.shared_image.get()
        payload = {'image': '', 'shape': ''}
    	
        shape = image.shape
        frame = cv2.imencode('.PNG', image)[1]
        encoded_image = base64.b64encode(frame)
        
        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape
        
        return payload

    def showNumpy(self, image):
        processed_image = np.stack((image,) * 3, axis=-1)
        self.shared_image.add(processed_image)

    def showPath(self, array):
        """Process the array(ideal path) to be sent to websocket"""
        with self.array_lock:
                strArray = ''.join(str(e) for e in array)

                # Remove unnecesary spaces in the array to avoid JSON syntax error in javascript
                strArray = re.sub(r"\[[ ]+", "[", strArray)
                strArray = re.sub(r"[ ]+", ", ", strArray)
                strArray = re.sub(r",[ ]+]", "]", strArray)
                strArray = re.sub(r",,", ",", strArray)
                strArray = re.sub(r"]\[", "],[", strArray)
                strArray = "[" + strArray + "]"

                self.array = strArray

    def getTargetPose(self):
        if (self.worldXY != None):
            return [self.worldXY[1], self.worldXY[0]]
        else:
            return None

    def update_gui(self):
        """Updates the GUI with the latest map information."""
        # Payload Image Message
        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        self.payload["array"] = self.array
        # Payload Map Message
        pos_message1 = self.map.getTaxiCoordinates()
        # print(self.pose3d_object.getPose3d())
        ang_message = self.map.getTaxiAngle()
        pos_message = str(pos_message1 + ang_message)
        # print("pos2 : {} ,  ang : {}".format(pos_message,ang_message))
        self.payload["map"] = pos_message

        message = json.dumps(self.payload)
        if self.client:
            try:
                self.client.send(message)
            except Exception as e:
                print(f"Error sending message: {e}")

        return list(pos_message1)

    def on_message(self, ws, message):
        """Handles incoming messages from the websocket client."""
        if message.startswith("#ack"):
            with self.ack_lock:
                self.ack=True
        elif (message[:5] == "#pick"):
            data = eval(message[5:])
            self.mapXY = data
            x, y = self.mapXY
            worldx, worldy = self.map.gridToWorld(x, y)
            self.worldXY = [worldx, worldy]
            print("World : {}".format(self.worldXY))

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