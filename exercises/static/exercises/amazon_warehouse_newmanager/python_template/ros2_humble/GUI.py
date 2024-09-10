import re
import json
import threading
import matplotlib.pyplot as plt

from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from map import Map
from HAL import getPose3d
from console_interfaces.general.console import start_console


class GUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.array_lock = threading.Lock()
        self.array = None

        # Payload vars
        self.payload = {'map': '', 'array': ''}
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)
        self.map = Map(getPose3d)

        self.start()

    # Prepares and sends a map to the websocket server
    def update_gui(self):
        self.payload["array"] = self.array

        # Payload Map Message
        pos_message = self.map.getRobotCoordinates()
        ang_message = self.map.getRobotAngle()
        pos_message = str(pos_message + ang_message)
        self.payload["map"] = pos_message

        message = json.dumps(self.payload)
        self.send_to_client(message)

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
gui = GUI(host)

# Redirect the console
start_console()

def showPath(array):
    return gui.showPath(array)

def getMap(url):
    return gui.getMap(url)