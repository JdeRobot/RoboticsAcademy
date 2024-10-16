import json

from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from console_interfaces.general.console import start_console
from lap import Lap
from map import Map
from HAL import getLaserData, getPose3d

# Graphical User Interface Class

class GUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # Payload vars
        self.payload = {'lap': '', 'map': ''}
        self.map = Map(getLaserData, getPose3d)
        self.lap = Lap(self.map)
        
        self.start()

    # Prepares and sends a map to the websocket server
    def update_gui(self):

        lapped = self.lap.check_threshold()
        lap_message = ""
        if(lapped != None):
            self.payload["lap"] = str(lapped)

        # Payload Map Message
        map_message = self.map.get_json_data()
        self.payload["map"] = map_message

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def showForces(self, vec1, vec2, vec3):
        """Function for student to call"""
        self.map.setCar(vec1[0], vec1[1])
        self.map.setObs(vec2[0], vec2[1])
        self.map.setAvg(vec3[0], vec3[1])

    def showLocalTarget(self, newVec):
        """Function for student to call"""
        self.map.setTargetPos(newVec[0], newVec[1])

    def reset_gui(self):
        """Resets the GUI to its initial state."""
        self.map.reset()
        self.lap.reset()

host = "ws://127.0.0.1:2303"
gui = GUI(host)

# Redirect the console
start_console()

def showForces(vec1, vec2, vec3):
    gui.showForces(vec1, vec2, vec3)

def showLocalTarget(newVec):
    return gui.showLocalTarget(newVec)

#TODO: change this to another file, not GUI

def getNextTarget():
    return gui.map.getNextTarget()

def setTargetx(x):
    gui.map.targetx = x

def setTargety(y):
    gui.map.targety = y