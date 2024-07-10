import json
import threading
import time
import math
import websocket
import rclpy
import matplotlib.pyplot as plt
from src.manager.ram_logging.log_manager import LogManager
from console import start_console
from map import Map
from HAL import getPose3d

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

        self.running = True

        self.host = host
        self.node = rclpy.create_node("node")

        # Payload vars
        self.payload = {'map': '', 'user': '', 'particles': ''}
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)
        self.map = Map(getPose3d)

        # Particles
        self.particles = []
        # User position
        self.user_position = (0, 0)
        self.user_angle = (0, 0)

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

        # Payload Map Message
        pos_message = self.map.getRobotCoordinates()
        if pos_message == self.init_coords:
            pos_message = self.start_coords
        ang_message = self.map.getRobotAngle()
        pos_message = str(pos_message + ang_message)
        self.payload["map"] = pos_message

        # Payload User Message
        pos_message_user = self.user_position
        ang_message_user = self.user_angle
        pos_message_user = pos_message_user + ang_message_user
        pos_message_user = str(pos_message_user)
        self.payload["user"] = pos_message_user

        # Payload Particles Message
        if self.particles:
            self.payload["particles"] = json.dumps(self.particles)
        else:
            self.payload["particles"] = json.dumps([])

        message = json.dumps(self.payload)
        if self.client:
            try:
                self.client.send(message)
            except Exception as e:
                LogManager.logger.info(f"Error sending message: {e}")

    def showPosition(self, x, y, angle):
        angle += math.pi
        ay = math.cos(-angle) - math.sin(-angle)
        ax = math.sin(-angle) + math.cos(-angle)
        scale_y = 15
        offset_y = 63
        y = scale_y * y + offset_y
        scale_x = -30
        offset_x = 171
        x = scale_x * x + offset_x
        self.user_position = x, y
        self.user_angle = (ax, ay)

    def showParticles(self, particles):
        if particles:
            self.particles = particles
            scale_y = 15
            offset_y = 63
            scale_x = -30
            offset_x = 171
            for particle in self.particles:
                particle[1] = scale_y * particle[1] + offset_y
                particle[0] = scale_x * particle[0] + offset_x
        else:
            self.particles = []

    def getMap(self, url):
        return plt.imread(url)

    def poseToMap(self, x_prime, y_prime, yaw_prime):
        scale_x = 1024 / 9.885785
        offset_x = 5.650662
        x = 1181 - (x_prime + offset_x) * scale_x
        scale_y = 1024 / 9.75819
        offset_y = 4.088577
        y = 24 + (y_prime + offset_y) * scale_y
        yaw = yaw_prime
        return [x, y, yaw]

    def mapToPose(self, x, y, yaw):
        scale_x = 1024 / 9.885785
        offset_x = 5.650662
        x = ((1181 - x) / scale_x) - offset_x
        scale_y = 1024 / 9.75819
        offset_y = 4.088577
        y = ((-24 + y) / scale_y) - offset_y
        return [x, y, yaw]

host = "ws://127.0.0.1:2303"
gui = ThreadingGUI(host)

# Redirect the console
start_console()

# Expose to the user
def showPosition(x, y, angle):
    gui.showPosition(x, y, angle)

def showParticles(particles):
    gui.showParticles(particles)

def getMap(url):
    return gui.getMap(url)

def poseToMap(x_prime, y_prime, yaw_prime):
    return gui.poseToMap(x_prime, y_prime, yaw_prime)

def mapToPose(x, y, yaw):
    return gui.mapToPose(x, y, yaw)