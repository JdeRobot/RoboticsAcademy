import cv2
import base64
import json
import threading
import time
import websocket
import rclpy
from src.manager.ram_logging.log_manager import LogManager
from hal_interfaces.general.odometry import OdometryNode
from console import start_console
from lap import Lap
from map import Map

# Graphical User Interface Class

class ThreadingGUI:

    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):

        # ROS 2 init
        if not rclpy.ok():
            rclpy.init()

        # Execution control vars
        self.out_period = 1.0 / freq

        self.ack = True
        self.ack_frontend = False
        self.ack_lock = threading.Lock()

        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.running = True

        self.host = host
        self.node = rclpy.create_node("node")

        # Payload vars
        self.payload = {'image': '','lap': '', 'map': ''}
        self.circuit = "simple"
        # TODO: maybe move this to HAL and have it be hybrid
        pose3d_object = OdometryNode("/odom")
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(pose3d_object)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        self.lap = Lap(pose3d_object)
        self.map = Map(pose3d_object, self.circuit)

        # Initialize and start the WebSocket client thread
        threading.Thread(target=self.run_websocket, daemon=True).start()

        # Initialize and start the image sending thread (GUI out thread)
        threading.Thread(
            target=self.gui_out_thread, name="gui_out_thread", daemon=True
        ).start()

    # Init websocket client
    def run_websocket(self):
        self.client = websocket.WebSocketApp(self.host, on_message=self.gui_in_thread)
        self.client.run_forever(ping_timeout=None, ping_interval=0)

    # Process incoming messages to the GUI
    def gui_in_thread(self, ws, message):

        # In this case, incoming msgs can only be acks
        if "ack" in message:
            with self.ack_lock:
                self.ack = True
                self.ack_frontend = True

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

    # Prepares and sends a map to the websocket server
    def update_gui(self):

        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)
        
        # Payload Lap Message
        lapped = self.lap.check_threshold()
        self.payload["lap"] = ""
        if(lapped != None):
            self.payload["lap"] = str(lapped)
            
        # Payload Map Message
        pos_message = str(self.map.getFormulaCoordinates())
        self.payload["map"] = pos_message
        
        message = json.dumps(self.payload)
        if self.client:
            try:
                self.client.send(message)
                # print(message)
            except Exception as e:
                LogManager.logger.info(f"Error sending message: {e}")

    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image_to_be_shown = self.image_to_be_shown

        image = image_to_be_shown
        payload = {'image': '', 'shape': ''}

        if not image_to_be_shown_updated:
            return payload

        shape = image.shape
        frame = cv2.imencode('.JPEG', image)[1]
        encoded_image = base64.b64encode(frame)

        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload
    
    # Function for student to call
    def showImage(self, image):
        with self.image_show_lock:
            self.image_to_be_shown = image
            self.image_to_be_shown_updated = True

host = "ws://127.0.0.1:2303"
gui = ThreadingGUI(host)

# Redirect the console
start_console()

# Expose to the user
def showImage(image):
    gui.showImage(image)