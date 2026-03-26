import json
import cv2
import base64
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from console_interfaces.general.console import start_console


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):

        # ROS 2 init
        if not rclpy.ok():
            rclpy.init()

        self.out_period = 1.0 / freq
        self.image = None
        self.image_lock = threading.Lock()
        self.ack = True
        self.ack_lock = threading.Lock()
        self.running = True
        self.host = host
        self.msg = {"image": ""}

        self.iteration_counter = 0
        self.last_time = time.time()
        self.ideal_cycle = 80
        self.real_time_factor = 0
        self.frequency_message = {"brain": "", "gui": "", "rtf": ""}
        self.fps = -1
        self.lat = -1
        self.keys_received = 0

        self.node = Node("webgui_person_control")
        self.pub = self.node.create_publisher(Twist, "/person/cmd_vel", 10)

        self.start()

    def gui_in_thread(self, ws, message):
        """Listen to messages and send control speeds"""

        # ACK del frontend
        if "ack" in message:
            with self.ack_lock:
                self.ack = True
            return

        if "start" in message:
            with self.ack_lock:
                self.ack_frontend = True
            return

        # Log teclas
        self.keys_received += 1
        twist = Twist()

        LINEAR_SPEED = 0.01
        ANGULAR_SPEED = 0.01

        # LINEAR
        if message == "key_w":
            twist.linear.x = LINEAR_SPEED
        elif message == "key_s":
            twist.linear.x = -LINEAR_SPEED
        elif message in ["key_w_up", "key_s_up"]:
            twist.linear.x = 0.0

        # ANGULAR
        if message == "key_a":
            twist.angular.z = ANGULAR_SPEED
        elif message == "key_d":
            twist.angular.z = -ANGULAR_SPEED
        elif message in ["key_a_up", "key_d_up"]:
            twist.angular.z = 0.0

        self.pub.publish(twist)

    # Process outcoming messages from the GUI
    def gui_out_thread(self):
        while self.running:
            start_time = time.time()

            # Check if a new image should be sent
            with self.ack_lock:
                with self.image_lock:
                    if self.ack and self.image is not None:
                        self.update_gui()
                        self.ack = False

            # Maintain desired frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, self.out_period - elapsed)
            time.sleep(sleep_time)

    def update_gui(self):
        if self.image is None:
            return

        _, encoded = cv2.imencode(".JPEG", self.image)
        payload = {
            "image": base64.b64encode(encoded).decode(),
            "shape": self.image.shape,
        }

        self.msg["image"] = json.dumps(payload)
        message = json.dumps(self.msg)
        self.send_to_client(message)

    def setImage(self, image):
        with self.image_lock:
            self.image = image


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

# Redirect the console
start_console()


# Expose the gui setImage function
def showImage(img):
    gui.setImage(img)
