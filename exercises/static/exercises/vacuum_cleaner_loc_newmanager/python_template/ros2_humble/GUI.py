import json
import cv2
import base64
import threading
import time
import websocket
from src.manager.ram_logging.log_manager import LogManager
from gazebo_msgs.srv import SetEntityState, GetEntityState
from shared.image import SharedImage
import matplotlib.pyplot as plt
import rclpy
from console import start_console
from map import Map
from HAL import getPose3d

from map import Map

# Graphical User Interface Class

# Matrix colors
red = [0, 0, 255]
orange = [0, 165, 255]
yellow = [0, 255, 255]
green = [0, 255, 0]
blue = [255, 0, 0]
indigo = [130, 0, 75]
violet = [211, 0, 148]

class ThreadingGUI:

    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):

        # ROS 2 init
        if not rclpy.ok():
            rclpy.init()

        # Execution control vars
        self.out_period = 1.0 / freq
        self.ack = True
        self.ack_lock = threading.Lock()
        self.running = True
        self.host = host
        self.node = rclpy.create_node("node")

        # Payload vars
        # self.msg = {"pos_msg": "", "ang_msg": ""}
        self.msg = {'map': '', 'nav': ''}
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)
        self.map = Map(getPose3d)

        # Image vars
        self.shared_image = SharedImage("guiimage")

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

    # Process outcoming messages from the GUI
    def gui_out_thread(self):
        while self.running:
            start_time = time.time()

            # Check if a new map should be sent
            with self.ack_lock:
                if self.ack and self.map is not None:
                    self.update_gui()
                    self.ack = False

            # Maintain desired frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, self.out_period - elapsed)
            time.sleep(sleep_time)

    # Update the gui
    def update_gui(self):
        # Payload Map Message
        pos_message = self.map.getRobotCoordinates()
        if (pos_message == self.init_coords):
            pos_message = self.start_coords
        ang_message = self.map.getRobotAngle()
        pos_message = str(pos_message + ang_message)
        self.msg["map"] = pos_message

        # Example Payload Navigation Data message (random data)
        # 4 colors supported (0, 1, 2, 3)
        #nav_mat = np.zeros((20, 20), int)
        #nav_mat[2, 1] = 1
        #nav_mat[3, 3] = 2
        #nav_mat[5,9] = 3
        #nav_message = str(nav_mat.tolist())
        payload = self.payloadImage()
        self.msg["image"] = json.dumps(payload)

        message = json.dumps(self.msg)
        if self.client:
            try:
                self.client.send(message)
            except Exception as e:
                print(f"Error sending message: {e}")


    # Prepares and sends a map to the websocket server
    # def send_map(self):

    #     # Get the necessary info
    #     pos_message = self.map.getRobotCoordinates()
    #     if pos_message == self.init_coords:
    #         pos_message = self.start_coords
    #     ang_message = self.map.getRobotAngle()

    #     self.msg["pos_msg"] = pos_message
    #     self.msg["ang_msg"] = ang_message
    #     message = json.dumps(self.msg)
    #     try:
    #         if self.client:
    #             self.client.send(message)
    #     except Exception as e:
    #         LogManager.logger.info(f"Error sending message: {e}")

    # encode the image data to be sent to websocket
    def payloadImage(self):

        image = self.shared_image.get()
        payload = {'image': '', 'shape': ''}
    	
        shape = image.shape
        frame = cv2.imencode('.PNG', image)[1]
        encoded_image = base64.b64encode(frame)
        
        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape
        
        return payload
    
    def process_colors(self, image):
        colored_image = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)

        # Grayscale for values < 128
        mask = image < 128
        colored_image[mask] = image[mask][:, None] * 2

        # Color lookup table
        color_table = {
            128: red,
            129: orange,
            130: yellow,
            131: green,
            132: blue,
            133: indigo,
            134: violet
        }

        for value, color in color_table.items():
            mask = image == value
            colored_image[mask] = color

        return colored_image

    # load the image data
    def showNumpy(self, image):
        self.shared_image.add(self.process_colors(image))

    def getMap(self, url):        
        return plt.imread(url)


host = "ws://127.0.0.1:2303"
gui = ThreadingGUI(host)

def showNumpy(image):
    gui.show(image)

# Redirect the console
start_console()