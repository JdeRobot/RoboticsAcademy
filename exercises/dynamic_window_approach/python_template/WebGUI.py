import json

from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from console_interfaces.general.console import start_console
from lap import Lap
from map import Map
from HAL import getLaserData, getPose3d


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # Payload vars
        self.payload = {"lap": "", "map": ""}
        self.map = Map(getLaserData, getPose3d)
        self.lap = Lap(self.map)

        self.start()

    # Prepares and sends a map to the websocket server
    def update_gui(self):
        lapped = self.lap.check_threshold()
        if lapped is not None:
            self.payload["lap"] = str(lapped)

        map_message = self.map.get_json_data()
        self.payload["map"] = map_message

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def showLocalTarget(self, newVec):
        """Function for student to call"""
        self.map.setTargetPos(newVec[0], newVec[1])

    # --- Métodos para ventana dinámica ---
    def showDynamicWindow(self, dynamic_window):
        self.map.showDynamicWindow(dynamic_window)

    def showBestVelocity(self, best_vw):
        self.map.showBestVelocity(best_vw)

    def reset_gui(self):
        """Resets the GUI to its initial state."""
        self.map.reset()
        self.lap.reset()


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

# Redirect the console
start_console()


def showLocalTarget(newVec):
    return gui.showLocalTarget(newVec)


def showDynamicWindow(dynamic_window):
    gui.showDynamicWindow(dynamic_window)


def showBestVelocity(best_vw):
    gui.showBestVelocity(best_vw)


def getNextTarget():
    return gui.map.getNextTarget()


def setTargetx(x):
    gui.map.targetx = x


def setTargety(y):
    gui.map.targety = y