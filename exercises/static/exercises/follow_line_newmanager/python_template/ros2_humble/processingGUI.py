#
#
#
# THIS IS A PROTOTYPE, DO NOT USE
#
#
#

import cv2
import base64
import json
import threading
import rclpy
from gui_interfaces.general.processing_gui import ProcessingGUI
from console_interfaces.general.console import start_console
from hal_interfaces.general.odometry import OdometryNode
from lap import Lap
from map import Map

# Graphical User Interface Class

class GUI(ProcessingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        # Payload vars
        self.payload = {'image': '','lap': '', 'map': ''}
        self.circuit = "simple"
        # TODO: maybe move this to HAL and have it be hybrid
        pose3d_object = OdometryNode("/odom")
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(pose3d_object)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        self.lap = Lap(pose3d_object)
        self.map = Map(pose3d_object, self.circuit)

        self.start_threads()

    # Prepares and sends a map to the websocket server
    def update_gui(self):

        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)
        
        # Payload Lap Message
        lapped = self.lap.check_threshold()
        self.payload["lap"] = ""
        if(lapped != None):
            self.payload["lap"] = str(lapped)
            
        # Payload Map Message
        pos_message = str(self.map.getFormulaCoordinates())
        self.payload["map"] = pos_message
        
        message = json.dumps(self.payload)
        self.send_to_client(message)

    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image_to_be_shown = self.image_to_be_shown

        image = image_to_be_shown
        payload = {'image': '', 'shape': ''}

        if not image_to_be_shown_updated:
            return payload

        shape = image.shape
        frame = cv2.imencode('.JPEG', image)[1]
        encoded_image = base64.b64encode(frame)

        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload
    
    # Function for student to call
    def showImage(self, image):
        with self.image_show_lock:
            self.image_to_be_shown = image
            self.image_to_be_shown_updated = True

host = "ws://127.0.0.1:2303"
gui = GUI(host)
gui.start() # Needed to start the process

# Redirect the console
start_console()

# Expose to the user
def showImage(image):
    gui.showImage(image)