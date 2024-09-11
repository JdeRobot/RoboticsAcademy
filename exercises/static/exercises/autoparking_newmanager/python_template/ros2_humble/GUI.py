import json

from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from console_interfaces.general.console import start_console
from map import Map
from HAL import getFrontLaserData, getRightLaserData, getBackLaserData

# Graphical User Interface Class

class GUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # Payload vars
        self.payload = {'map': ''}
        self.map = Map(getFrontLaserData, getRightLaserData, getBackLaserData)

        self.start()

    # Prepares and sends a map to the websocket server
    def update_gui(self):

        map_message = self.map.get_json_data()
        self.payload["map"] = map_message
        message = json.dumps(self.payload)
        self.send_to_client(message)


    def reset_gui(self):
        """Resets the GUI to its initial state."""
        self.map.reset()

host = "ws://127.0.0.1:2303"
gui = GUI(host)

# Redirect the console
start_console()
