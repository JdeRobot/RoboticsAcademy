from datetime import datetime
import json
import subprocess
import rclpy
import threading
import time
import websocket
from src.manager.ram_logging.log_manager import LogManager


class MeasuringThreadingGUI:
    """ GUI interface using threading and measuring RTF data:
        
        self.start() needs to be called at the end of the init method\n
        The update_gui(self) method needs to be implemented
    """

    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):

        # ROS 2 init
        if not rclpy.ok():
            rclpy.init()

        # Execution control vars
        self.out_period = 1.0 / freq

        self.ack = True
        self.ack_frontend = False
        self.ack_lock = threading.Lock()

        self.ideal_cycle = 80
        self.real_time_factor = 0
        self.frequency_message = {'brain': '', 'gui': '', 'rtf': ''}
        self.iteration_counter = 0

        self.running = True

        self.host = host
        self.node = rclpy.create_node("node")
    
    def start(self):
        # Initialize and start the WebSocket client thread
        threading.Thread(target=self.run_websocket, daemon=True).start()
        
        # Initialize and start the RTF thread
        self.rtf_thread = threading.Thread(target=self.get_real_time_factor).start()

        # Initialize and start the Frequency thread
        self.frequency_thread = threading.Thread(target=self.measure_and_send_frequency).start()

        # Initialize and start the image sending thread (GUI out thread)
        threading.Thread(
            target=self.gui_out_thread, name="gui_out_thread", daemon=True
        ).start()

    # Init websocket client
    def run_websocket(self):
        while self.running:
            self.client = websocket.WebSocketApp(self.host, on_message=self.gui_in_thread)
            self.client.run_forever(ping_timeout=None, ping_interval=0)

    def get_real_time_factor(self):
        """Continuously calculates the real-time factor."""
        while self.running:
            time.sleep(2)
            args = ["gz", "stats", "-p"]
            stats_process = subprocess.Popen(args, stdout=subprocess.PIPE)
            with stats_process.stdout:
                for line in iter(stats_process.stdout.readline, b''):
                    stats_list = [x.strip() for x in line.split(b',')]
                    self.real_time_factor = stats_list[0].decode("utf-8")

    def measure_and_send_frequency(self):
        """Measures and sends the frequency of GUI updates and brain cycles."""
        previous_time = datetime.now()
        while self.running:
            time.sleep(2)
            current_time = datetime.now()
            dt = current_time - previous_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            previous_time = current_time
            measured_cycle = ms / self.iteration_counter if self.iteration_counter > 0 else 0
            self.iteration_counter = 0
            brain_frequency = round(1000 / measured_cycle, 1) if measured_cycle != 0 else 0
            gui_frequency = round(1000 / self.ideal_cycle, 1)
            self.frequency_message = {'brain': brain_frequency, 'gui': gui_frequency, 'rtf': self.real_time_factor}
            message = json.dumps(self.frequency_message)

            self.send_to_client(message)

    # Process incoming messages to the GUI
    def gui_in_thread(self, ws, message):

        # In this case, incoming msgs can only be acks
        if "ack" in message:
            with self.ack_lock:
                self.ack = True
                self.ack_frontend = True
        else:
            LogManager.logger.error("Unsupported msg")
    
    def update_gui(self):
        """Prepares the data and calls the following method at the end to send it:\n
            · send_to_client(data)
        """
        pass

    # Process outcoming messages from the GUI
    def gui_out_thread(self):
        while self.running:
            start_time = time.time()
            self.iteration_counter += 1

            # Check if a new map should be sent
            with self.ack_lock:
                if self.ack:
                    self.update_gui()
                    if self.ack_frontend: 
                        self.ack = False

            # Maintain desired frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, self.out_period - elapsed)
            time.sleep(sleep_time)

    def send_to_client(self, msg):
        if self.client:
            try:
                self.client.send(msg)
            except Exception as e:
                LogManager.logger.info(f"Error sending message: {e}")