import rclpy
import threading
import time
import websocket
import multiprocessing
from src.manager.ram_logging.log_manager import LogManager

class ProcessingGUI(multiprocessing.Process):

    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):
        super(ProcessingGUI, self).__init__()

        # ROS 2 init
        if not rclpy.ok():
            rclpy.init()

        # Execution control vars
        self.out_period = 1.0 / freq

        self.ack = True
        self.ack_frontend = False
        self.ack_lock = threading.Lock()

        # self.client = RawValue(c_void_p, None)

        self.running = True

        self.host = host
        self.node = rclpy.create_node("node")
    
    def start_threads(self):
        # Initialize and start the WebSocket client thread
        threading.Thread(target=self.run_websocket, daemon=True).start()

        # Initialize and start the image sending thread (GUI out thread)
        threading.Thread(
            target=self.gui_out_thread, name="gui_out_thread", daemon=True
        ).start()

    # Init websocket client
    def run_websocket(self):
        while self.running:
            self.client = websocket.WebSocketApp(self.host, on_message=self.gui_in_thread)
            self.client.run_forever(ping_timeout=None, ping_interval=0)

    # Process incoming messages to the GUI
    def gui_in_thread(self, ws, message):

        # In this case, incoming msgs can only be acks
        if "ack" in message:
            with self.ack_lock:
                self.ack = True
                self.ack_frontend = True
        else:
            LogManager.logger.error("Unsupported msg")

    # Process outcoming messages from the GUI
    def gui_out_thread(self):
        while self.running:
            start_time = time.time()

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
