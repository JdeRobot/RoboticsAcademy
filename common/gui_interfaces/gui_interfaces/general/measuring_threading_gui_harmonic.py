from datetime import datetime
import json
import subprocess
import threading
import time
from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosedOK
import asyncio
from threading import Timer
import re
import sys


from gz.transport import Node
from gz.msgs.world_stats_pb2 import WorldStatistics

sys.path.insert(0, "/RoboticsApplicationManager")

from robotics_application_manager import LogManager


class MeasuringThreadingGUI:
    """GUI interface using threading and measuring RTF data:

    self.start() needs to be called at the end of the init method\n
    The update_gui(self) method needs to be implemented
    """

    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0, world_name="default"):

        # Execution control vars
        self.out_period = 1.0 / freq

        self.ack = True
        self.ack_frontend = False
        self.ack_lock = threading.Lock()

        self.ideal_cycle = 80
        self.real_time_factor = 0
        self.frequency_message = {
            "brain": "",
            "gui": "",
            "rtf": "",
            "fps": "",
            "lat": "",
        }
        self.iteration_counter = 0
        self.fps = -1
        self.lat = -1

        self.host = host

        self.world_name = world_name
        self.client = None

    def start(self):
        # Initialize and start the WebSocket client thread
        threading.Thread(
            target=self.launch_websocket, name="websocket_thread", daemon=True
        ).start()

        # Initialize and start the RTF thread
        threading.Thread(
            target=self.get_real_time_factor, name="rtf_thread", daemon=True
        ).start()

        # Initialize and start the Frequency thread
        threading.Thread(
            target=self.launch_measure_and_send_frequency, name="frequency_thread", daemon=True
        ).start()

        # Initialize and start the image sending thread (GUI out thread)
        threading.Thread(
            target=self.launch_gui_out_thread, name="gui_out_thread", daemon=True
        ).start()

    def rtf_callback(self, msg: WorldStatistics):
        self.real_time_factor = round(msg.real_time_factor, 2)

    def launch_gui_out_thread(self):
        asyncio.run(self.gui_out_thread())

    def launch_measure_and_send_frequency(self):
        asyncio.run(self.measure_and_send_frequency())

    def launch_websocket(self):
        asyncio.run(self.run_websocket())

    async def run_websocket(self):
        try:
          async with connect(self.host) as websocket:
            self.client = websocket

            async for raw_msg in websocket:
              self.gui_in_thread(websocket, raw_msg)
        except ConnectionClosedOK as e:
            pass
        finally:
            self.client = None

    def get_real_time_factor(self):
        """Continuously calculates the real-time factor."""

        node = Node()
        node.subscribe(
            WorldStatistics, f"/world/{self.world_name}/stats", self.rtf_callback
        )

        while True:
            time.sleep(0.001)

    async def measure_and_send_frequency(self):
        """Measures and sends the frequency of GUI updates and brain cycles."""
        previous_time = datetime.now()
        while True:
            time.sleep(2)
            current_time = datetime.now()
            dt = current_time - previous_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            previous_time = current_time
            measured_cycle = (
                ms / self.iteration_counter if self.iteration_counter > 0 else 0
            )
            self.iteration_counter = 0
            brain_frequency = (
                round(1000 / measured_cycle, 1) if measured_cycle != 0 else 0
            )
            gui_frequency = round(1000 / self.ideal_cycle, 1)
            self.frequency_message = {
                "brain": brain_frequency,
                "gui": gui_frequency,
                "rtf": self.real_time_factor,
                "fps": self.fps,
                "lat": self.lat,
            }
            message = json.dumps(self.frequency_message)

            await self.send_to_client(message)

    # Process incoming messages to the GUI
    def gui_in_thread(self, ws, message):

        # In this case, incoming msgs can only be acks
        if "ack" in message:
            with self.ack_lock:
                self.ack = True
        elif "start" in message:
            with self.ack_lock:
                self.ack_frontend = True
        else:
            LogManager.logger.error("Unsupported msg")

    async def update_gui(self):
        """Prepares the data and calls the following method at the end to send it:\n
        · send_to_client(data)
        """
        pass

    # Process outcoming messages from the GUI
    async def gui_out_thread(self):
        while True:
            start_time = time.time()
            self.iteration_counter += 1

            # Check if a new map should be sent
            with self.ack_lock:
                if self.ack_frontend and self.ack:
                    await self.update_gui()
                    self.ack = False

            # Maintain desired frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, self.out_period - elapsed)
            time.sleep(sleep_time)

    async def send_to_client(self, msg):
        if self.client:
            try:
                await self.client.send(msg)
            except Exception as e:
                LogManager.logger.info(f"Error sending message: {e}")
