#
#
#
# THIS IS A PROTOTYPE, DO NOT USE
#
#
#

import json
import cv2
import base64
import threading
import time
from gazebo_msgs.srv import SetEntityState, GetEntityState
import rclpy
from math import cos, sin, atan2

from gui_interfaces.general.processing_gui import ProcessingGUI
from gui_interfaces.general.threading_gui import ThreadingGUI
from console_interfaces.general.console import start_console

class GUI(ProcessingGUI):

    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):
        super(ProcessingGUI, self).__init__()
        # ROS 2 init
        if not rclpy.ok():
            rclpy.init()

        # Execution control vars
        self.out_period = 1.0 / freq
        self.image = None
        self.image_lock = threading.Lock()
        self.ack = True
        self.ack_lock = threading.Lock()
        self.running = True

        self.host = host
        self.msg = {"image": ""}
        self.node = rclpy.create_node("node")

        # Initialize the services
        self.set_client = self.node.create_client(
            SetEntityState, "/follow_person/set_entity_state"
        )
        while not self.set_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        self.set_request = SetEntityState.Request()

        self.get_client = self.node.create_client(
            GetEntityState, "/follow_person/get_entity_state"
        )
        while not self.get_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        self.get_request = GetEntityState.Request()

        # Initialize and start the WebSocket client thread
        self.start_threads()

    # Process incoming messages to the GUI
    def gui_in_thread(self, ws, message):

        # In this case, messages can be either acks or key strokes
        if "ack" in message:
            with self.ack_lock:
                self.ack = True
        else:
            # Get the current pose
            self.get_request.name = "PersonToControl"
            self.get_request.reference_frame = "world"
            get_future = self.get_client.call_async(self.get_request)
            rclpy.spin_until_future_complete(self.node, get_future)
            pose = get_future.result().state.pose

            # Define movement and rotation parameters
            mov_dist = 0.1  # meters (default for forward movement)
            rot_angle = 0.17 # radians (default for left rotation)

            # Check for movement direction 
            if  "key_s" in message:     
                mov_dist *= -1          # reverse for backward movement
            if "key_d" in message:      
                rot_angle *= -1         # reverse for right rotation

            # Update accordingly
            if "key_w" in message or "key_s" in message:   # forward or backward movement
                siny_cosp = 2 * (pose.orientation.w * pose.orientation.z - pose.orientation.x * pose.orientation.y)
                cosy_cosp = 1 - 2 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z)
                yaw = atan2(siny_cosp, cosy_cosp)
                pose.position.x += mov_dist * sin(yaw)
                pose.position.y += -mov_dist * cos(yaw)
            elif "key_a" in message or "key_d" in message:  # turning movement
                w = pose.orientation.w * cos(rot_angle / 2) - pose.orientation.z * sin(rot_angle / 2)
                x = pose.orientation.x * cos(rot_angle / 2) + pose.orientation.y * sin(rot_angle / 2)
                y = pose.orientation.y * cos(rot_angle / 2) - pose.orientation.x * sin(rot_angle / 2)
                z = pose.orientation.w * sin(rot_angle / 2) + pose.orientation.z * cos(rot_angle / 2)
                pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = w, x, y, z

            # Send the new pose
            self.set_request.state.name = "PersonToControl"
            self.set_request.state.pose = pose
            self.set_request.state.reference_frame = "world"
            set_future = self.set_client.call_async(self.set_request)
            rclpy.spin_until_future_complete(self.node, set_future)

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

    # Prepares and send image to the websocket server
    def update_gui(self):
        
        _, encoded_image = cv2.imencode(".JPEG", self.image)
        payload = {
            "image": base64.b64encode(encoded_image).decode("utf-8"),
            "shape": self.image.shape,
        }
        self.msg["image"] = json.dumps(payload)
        message = json.dumps(self.msg)
        self.send_to_client(message)

    # Function to set the next image to be sent
    def setImage(self, image):
        with self.image_lock:
            self.image = image


host = "ws://127.0.0.1:2303"
gui = GUI(host)
gui.start()

# Redirect the console
start_console()

# Expose the gui setImage function
def showImage(img):
    gui.setImage(img)
