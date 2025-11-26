import json
import math

from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console
from HAL import getLidarData

# Graphical User Interface Class


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # Payload vars
        self.payload = {"lidar": ""}
        # self.map = Map(LidarNode)

        self.start()

    # Prepares and sends a map to the websocket server
    def update_gui(self):
        pass

        # map_message = self.map.get_json_data()
        lidar = getLidarData()
        points = []
        color = (20, 20, 255)
        if lidar:
            for i in range(len(lidar.points)):
                is_valid = True
                for j in range(3):
                    dist = lidar.points[i][j]
                    if dist == float("inf") or dist == float("-inf"):
                        is_valid = False
                if is_valid:
                    x, y, z = lidar.points[i]
                    points.append(
                        (float(x * 10), float((z + 1.75) * 10), float(-y * 10)) + color
                    )
        self.payload["lidar"] = json.dumps(points)
        message = json.dumps(self.payload)
        self.send_to_client(message)

    def reset_gui(self):
        """Resets the GUI to its initial state."""
        self.map.reset()


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

# Redirect the console
start_console()
