import json
import cv2
import base64
import threading
import sys
import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image

from gui_interfaces.general.measuring_threading_gui_no_sim import MeasuringThreadingGUI
from console_interfaces.general.console import start_console
from hal_interfaces.general.camera import CameraNode
import numpy as np


class InputImagePublisher(Node):
    """
    Minimal direct ROS2 publisher for browser input images.
    Publishes to /input/image_raw without using HAL high-level helpers.
    """

    def __init__(self, topic_name="/input/image_raw"):
        super().__init__("visual_object_detection_input_publisher")
        self.publisher = self.create_publisher(Image, topic_name, 10)

    def publish_image(self, image):
        if image is None:
            return

        try:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height = image.shape[0]
            msg.width = image.shape[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = image.shape[1] * 3
            msg.data = image.tobytes()
            self.publisher.publish(msg)

        except Exception as e:
            print(f"Error publishing input image: {e}", file=sys.stderr)


# Graphical User Interface Class
class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # Execution control vars
        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.payload = {"image": "", "shape": ""}
        self.frame_rgb = None
        self.frame_rgb_lock = threading.Lock()

        self.has_received_img = False

        # Initialize ROS2 if not already initialized
        if not rclpy.ok():
            rclpy.init()

        self.camera_node = None
        self.input_publisher_node = None
        self.auto_image_mode = False
        self.executor = None
        self.executor_thread = None
        self.auto_image_thread = None

        self._setup_auto_mode()

        # Start ROS2 executor and image display thread if nodes created
        if self.camera_node and self.input_publisher_node:
            self._start_ros2_threads()

        self.start()

    def _setup_auto_mode(self):
        """Set up automatic subscription for /webgui_image and direct publisher for /input/image_raw"""
        try:

            self.camera_node = CameraNode("/webgui_image")

            self.input_publisher_node = InputImagePublisher("/input/image_raw")

            self.auto_image_mode = True
            print("ROS2 mode enabled: Subscribed to /webgui_image")
            print("ROS2 mode enabled: Publishing to /input/image_raw")

        except Exception as e:
            print(f"ROS2 mode disabled: {e}", file=sys.stderr)
            self.camera_node = None
            self.input_publisher_node = None
            self.auto_image_mode = False

    def _start_ros2_threads(self):
        """Start ROS2 executor and image display thread"""
        try:
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.camera_node)
            self.executor.add_node(self.input_publisher_node)

            self.executor_thread = threading.Thread(
                target=self.executor.spin, daemon=True, name="webgui_ros2_executor"
            )
            self.executor_thread.start()

            self.auto_image_thread = threading.Thread(
                target=self._unified_image_loop,
                daemon=True,
                name="webgui_image_display",
            )
            self.auto_image_thread.start()

        except Exception as e:
            print(f"Error starting ROS2 threads: {e}", file=sys.stderr)

    def _unified_image_loop(self):
        """Unified image handling loop for ROS2 subscriber"""
        while True:
            try:
                if self.camera_node:
                    image = self.camera_node.getImage()
                    if image is not None:
                        self.showImage(image.data)

                threading.Event().wait(0.033)  # ~30 FPS

            except Exception as e:
                print(f"Error in image display loop: {e}", file=sys.stderr)
                threading.Event().wait(1.0)

    # Process incoming messages to the GUI
    def gui_in_thread(self, ws, message):
        # time frame size
        time_frame_size = 20

        if "ack" in message:
            with self.ack_lock:
                self.ack = True
        elif "start" in message:
            with self.ack_lock:
                self.ack_frontend = True

        if "pick" in message:
            self.has_received_img = True

            # Image from the frontend
            base64_buffer = message[4:-time_frame_size]
            time = message[-time_frame_size:]

            if base64_buffer.startswith("data:image/jpeg;base64,"):
                base64_buffer = base64_buffer[len("data:image/jpeg;base64,") :]

            image_data = base64.b64decode(base64_buffer)

            nparr = np.frombuffer(image_data, np.uint8)

            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            with self.frame_rgb_lock:
                self.frame_rgb = img
                ack_message = {"ack_img": "ack", "time": time}
                self.send_to_client(json.dumps(ack_message))

            if self.input_publisher_node is not None:
                self.input_publisher_node.publish_image(img)

        if "introspection" in message:
            info = message[len("introspection:") :]
            self.fps, self.lat = info.split("/")

    # Prepares and sends a map to the websocket server
    def update_gui(self):

        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        message = json.dumps(self.payload)
        self.send_to_client(message)

        if not self.has_received_img:
            ack_message = {"ack_img": "ack", "time": ""}
            self.send_to_client(json.dumps(ack_message))

    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image_to_be_shown = self.image_to_be_shown

        image = image_to_be_shown
        payload = {"image": "", "shape": ""}

        if not image_to_be_shown_updated:
            return payload

        shape = image.shape
        frame = cv2.imencode(".JPEG", image)[1]
        encoded_image = base64.b64encode(frame)

        payload["image"] = encoded_image.decode("utf-8")
        payload["shape"] = shape

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload

    # Function for student to call
    def showImage(self, image):
        with self.image_show_lock:
            self.image_to_be_shown = image
            self.image_to_be_shown_updated = True

    def getImage(self):
        with self.frame_rgb_lock:
            return self.frame_rgb


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

# Redirect the console
start_console()


# Expose the user functions
def showImage(image):
    gui.showImage(image)


def getImage():
    return gui.getImage()
