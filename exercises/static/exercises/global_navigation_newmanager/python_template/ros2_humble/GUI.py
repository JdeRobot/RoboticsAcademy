import cv2
import base64
import re
import json
import threading
import numpy as np

from gui_interfaces.general.threading_gui import ThreadingGUI
from console_interfaces.general.console import start_console
from shared.image import SharedImage
from map import Map
from HAL import getPose3d

# Graphical User Interface Class

class GUI(ThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.array = None
        self.array_lock = threading.Lock()
        self.mapXY = None
        self.worldXY = None

        # Payload vars
        self.payload = {'image': '', 'map': '', 'array': ''}
        self.shared_image = SharedImage("numpyimage")
        self.map = Map(getPose3d)

        self.start()

    # Process incoming messages to the GUI
    def gui_in_thread(self, ws, message):

        # In this case, incoming msgs can only be acks
        if "ack" in message:
            with self.ack_lock:
                self.ack = True
                self.ack_frontend = True
        elif "pick" in message:
            data = eval(message[4:])
            self.mapXY = data
            x, y = self.mapXY
            worldx, worldy = self.map.gridToWorld(x, y)
            self.worldXY = [worldx, worldy]
            print(f"World : {self.worldXY}")

    # Prepares and sends a map to the websocket server
    def update_gui(self):

        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        self.payload["array"] = self.array
        # Payload Map Message
        pos_message1 = self.map.getTaxiCoordinates()
        ang_message = self.map.getTaxiAngle()
        pos_message = str(pos_message1 + ang_message)
        self.payload["map"] = pos_message

        message = json.dumps(self.payload)
        self.send_to_client(message)

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

            # Remove unnecessary spaces in the array to avoid JSON syntax error in JavaScript
            strArray = re.sub(r"\[[ ]+", "[", strArray)
            strArray = re.sub(r"[ ]+", ", ", strArray)
            strArray = re.sub(r",[ ]+]", "]", strArray)
            strArray = re.sub(r",,", ",", strArray)
            strArray = re.sub(r"]\[", "],[", strArray)
            strArray = "[" + strArray + "]"

            self.array = strArray

    def getTargetPose(self):
        if self.worldXY is not None:
            return [self.worldXY[1], self.worldXY[0]]
        else:
            return None

    def getMap(self, url):
        return self.map.getMap(url)
    
    def rowColumn(self, pose):
        return self.map.rowColumn(pose)

    def reset_gui(self):
        """Resets the GUI to its initial state."""
        print("Resetting image")
        image = [[0 for x in range(400)] for y in range(400)]
        self.showNumpy(np.clip(image, 0, 255).astype('uint8'))
        self.map.reset()

host = "ws://127.0.0.1:2303"
gui = GUI(host)

# Redirect the console
start_console()

# Expose to the user
def payloadImage():
    return gui.payloadImage()

def showNumpy(image):
    gui.showNumpy(image)

def showPath(array):
    gui.showPath(array)

def getTargetPose():
    return gui.getTargetPose()

def getMap(url):
    return gui.getMap(url)

def rowColumn(pose):
    return gui.rowColumn(pose)