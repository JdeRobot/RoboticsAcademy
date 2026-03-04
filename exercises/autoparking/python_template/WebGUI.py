import json
import math
import subprocess

from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console
from map import Map
from HAL import getFrontLaserData, getRightLaserData, getBackLaserData, getLidarData

# Graphical User Interface Class


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # Payload vars
        self.payload = {"map": "", "lidar": ""}
        self.map = Map(getFrontLaserData, getRightLaserData, getBackLaserData)

        self.mode = "Laser"

        # Check if lidar is publishing
        args = ["ros2", "topic", "info", "/prius_autoparking/pc2"]
        topic_info = subprocess.Popen(args, stdout=subprocess.PIPE)
        for line in topic_info.stdout:
            if line == b"Publisher count: 1\n":
                self.mode = "Lidar"

        self.start()

    def _downsample_lidar_points(self, lidar_points):
        voxel_size = 0.15
        max_points = 5000
        max_dist_sq = 2500.0
        color = (20, 20, 255)

        voxels = {}
        for x, y, z in lidar_points:
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            dist_sq = x * x + y * y + z * z
            if dist_sq >= max_dist_sq:
                continue

            key = (
                math.floor(x / voxel_size),
                math.floor(y / voxel_size),
                math.floor(z / voxel_size),
            )
            if key in voxels:
                continue

            voxels[key] = (x, y, z)
            if len(voxels) >= max_points:
                break

        points = []
        for key in sorted(voxels):
            x, y, z = voxels[key]
            points.append((float(x * 10), float((z + 1.75) * 10), float(-y * 10)) + color)

        return points

    # Prepares and sends a map to the websocket server
    def update_gui(self):

        if self.mode == "Laser":
            map_message = self.map.get_json_data()
            self.payload["map"] = map_message
            self.payload["lidar"] = ""
        elif self.mode == "Lidar":
            lidar = getLidarData()
            points = []
            if lidar:
                points = self._downsample_lidar_points(lidar.points)
            self.payload["lidar"] = json.dumps(points)
            self.payload["map"] = ""

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def reset_gui(self):
        """Resets the GUI to its initial state."""
        self.map.reset()


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

# Redirect the console
start_console()
