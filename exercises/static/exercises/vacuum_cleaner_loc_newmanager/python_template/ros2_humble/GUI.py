import json
import cv2
import base64
import numpy as np
from shared.image import SharedImage
from PIL import Image

from gui_interfaces.general.threading_gui import ThreadingGUI
from map import Map
from HAL import getPose3d
from console import start_console

# Graphical User Interface Class

# Matrix colors
red = [0, 0, 255]
orange = [0, 165, 255]
yellow = [0, 255, 255]
green = [0, 255, 0]
blue = [255, 0, 0]
indigo = [130, 0, 75]
violet = [211, 0, 148]

class GUI(ThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.shared_image = SharedImage("guiimage")

        # Payload vars
        self.payload = {'map': '', 'user': ''}
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)
        self.map = Map(getPose3d)

        self.start()

    # Prepares and sends a map to the websocket server
    def update_gui(self):

        pos_message = self.map.getRobotCoordinates()
        if (pos_message == self.init_coords):
            pos_message = self.start_coords
        ang_message = self.map.getRobotAngle()
        pos_message = str(pos_message + ang_message)
        self.payload["map"] = pos_message

        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        message = json.dumps(self.payload)
        self.send_to_client(message)

    # encode the image data to be sent to websocket
    def payloadImage(self):

        image = self.shared_image.get()
        payload = {'image': '', 'shape': ''}
    	
        shape = image.shape
        frame = cv2.imencode('.PNG', image)[1]
        encoded_image = base64.b64encode(frame)
        
        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape
        
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
            134: violet
        }

        for value, color in color_table.items():
            mask = image == value
            colored_image[mask] = color

        return colored_image

    # load the image data
    def showNumpy(self, image):
        self.shared_image.add(self.process_colors(image))

    def getMap(self, url):
        try:
        # Open with PIL
            with Image.open(url) as img:
                img = img.convert("RGB")
                img_array = np.array(img)
            return img_array
        except Exception as e:
            print(f"Error reading image from {url}: {e}")
            return None

    def reset_gui(self):
        self.map.reset()

host = "ws://127.0.0.1:2303"
gui = GUI(host)

# Redirect the console
start_console()

# Expose to the user
def showNumpy(image):
    gui.showNumpy(image)

def getMap(url):        
    return gui.getMap(url)