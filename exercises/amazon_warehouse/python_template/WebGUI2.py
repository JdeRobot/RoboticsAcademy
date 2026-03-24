import base64
import re
import json
import cv2
import threading
import numpy as np
import matplotlib.pyplot as plt

from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from map import Map
from HAL2 import getPose3d, getLiftState
from console_interfaces.general.console import start_console

# Matrix colors
red = [0, 0, 255]
orange = [0, 165, 255]
yellow = [0, 255, 255]
green = [0, 255, 0]
blue = [255, 0, 0]
indigo = [130, 0, 75]
violet = [211, 0, 148]


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.array_lock = threading.Lock()
        self.array = None

        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.payload = {"map": "", "array": "", "liftState": ""}

        self.map = Map(getPose3d)

        self.init_coords = self.map.getRobotCoordinates()
        self.start_coords = self.map.getRobotCoordinates()

        self.start()

    # Prepares and sends a map to the websocket server
    def update_gui(self):
        self.payload["array"] = self.array
        self.payload["liftState"] = getLiftState()

        # Payload Map Message
        pos_message = self.map.getRobotCoordinates()
        ang_message = self.map.getRobotAngle()
        pos_message = str(pos_message + ang_message)
        self.payload["map"] = pos_message

        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        message = json.dumps(self.payload)
        self.send_to_client(message)

        # Process the array(ideal path) to be sent to websocket

    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image_to_be_shown = self.image_to_be_shown

        image = image_to_be_shown
        payload = {"image": "", "shape": ""}

        if not image_to_be_shown_updated:
            return payload

        shape = image.shape
        frame = cv2.imencode(".JPEG", image)[1]
        encoded_image = base64.b64encode(frame)

        payload["image"] = encoded_image.decode("utf-8")
        payload["shape"] = shape

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload

    def process_colors(self, image):
        colored_image = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)

        # Grayscale for values < 128
        mask = image < 128
        colored_image[mask] = image[mask][:, None] * 2

        # Color lookup table
        color_table = {
            128: red,
            129: orange,
            130: yellow,
            131: green,
            132: blue,
            133: indigo,
            134: violet,
        }

        for value, color in color_table.items():
            mask = image == value
            colored_image[mask] = color

        return colored_image

    # load the image data
    def showNumpy(self, image):
        with self.image_show_lock:
            self.image_to_be_shown = self.process_colors(image)
            self.image_to_be_shown_updated = True

    def showPath(self, array):
        array_scaled = []
        for wp in array:
            array_scaled.append([wp[0] * 0.72, wp[1] * 0.545])

        print("Path array: " + str(array_scaled))
        self.array_lock.acquire()

        strArray = "".join(str(e) for e in array_scaled)
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
gui = WebGUI(host)

# Redirect the console
start_console()


def showPath(array):
    return gui.showPath(array)


def showNumpy(image):
    gui.showNumpy(image)


def getMap(url):
    return gui.getMap(url)
