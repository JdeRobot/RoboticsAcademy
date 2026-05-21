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


class BrowserInputImagePublisher(Node):
    """
    Minimal direct ROS2 publisher for browser input images.
    Publishes browser frames to /input/image_raw without using HAL high-level helpers.
    """

    def __init__(self, topic_name="/input/image_raw"):
        super().__init__("image_classification_input_publisher")
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


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # GUI display state
        self.display_image = None
        self.display_image_updated = False
        self.display_image_lock = threading.Lock()

        self.payload = {"image": "", "shape": ""}

        # Latest frame received from frontend
        self.latest_input_frame = None
        self.latest_input_frame_lock = threading.Lock()

        self.has_received_img = False

        # Initialize ROS2 if not already initialized
        if not rclpy.ok():
            rclpy.init()

        # ROS2 image bridge state
        self.webgui_image_subscriber = None
        self.input_image_publisher = None
        self.ros2_image_bridge_enabled = False
        self.executor = None
        self.executor_thread = None
        self.webgui_image_thread = None

        self._setup_ros2_image_bridge()

        # Start ROS2 executor and image forwarding thread if nodes were created
        if self.webgui_image_subscriber and self.input_image_publisher:
            self._start_ros2_threads()

        self.start()

    def _setup_ros2_image_bridge(self):
        """Set up ROS2 image bridge: subscribe to /webgui_image and publish to /input/image_raw."""
        try:
            self.webgui_image_subscriber = CameraNode("/webgui_image")
            self.input_image_publisher = BrowserInputImagePublisher("/input/image_raw")

            self.ros2_image_bridge_enabled = True
            print("ROS2 image bridge enabled")
            print("  /webgui_image  -> GUI display")
            print("  browser input  -> /input/image_raw")

        except Exception as e:
            print(f"ROS2 mode disabled: {e}", file=sys.stderr)
            self.webgui_image_subscriber = None
            self.input_image_publisher = None
            self.ros2_image_bridge_enabled = False

    def _start_ros2_threads(self):
        """Start ROS2 executor and GUI image forwarding thread."""
        try:
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.webgui_image_subscriber)
            self.executor.add_node(self.input_image_publisher)

            self.executor_thread = threading.Thread(
                target=self.executor.spin,
                daemon=True,
                name="webgui_ros2_executor",
            )
            self.executor_thread.start()

            self.webgui_image_thread = threading.Thread(
                target=self._webgui_image_forwarding_loop,
                daemon=True,
                name="webgui_image_display",
            )
            self.webgui_image_thread.start()

        except Exception as e:
            print(f"Error starting ROS2 threads: {e}", file=sys.stderr)

    def _webgui_image_forwarding_loop(self):
        """Read images from /webgui_image and forward them to the GUI display."""
        while True:
            try:
                if self.webgui_image_subscriber:
                    image = self.webgui_image_subscriber.getImage()
                    if image is not None:
                        self.showImage(image.data)

                threading.Event().wait(0.033)  # ~30 FPS

            except Exception as e:
                print(f"Error in image display loop: {e}", file=sys.stderr)
                threading.Event().wait(1.0)

    def gui_in_thread(self, ws, message):
        time_frame_size = 20

        if "ack" in message:
            with self.ack_lock:
                self.ack = True
        elif "start" in message:
            with self.ack_lock:
                self.ack_frontend = True

        if "pick" in message:
            self.has_received_img = True

            # Image received from the frontend
            base64_buffer = message[4:-time_frame_size]
            time = message[-time_frame_size:]

            if base64_buffer.startswith("data:image/jpeg;base64,"):
                base64_buffer = base64_buffer[len("data:image/jpeg;base64,") :]

            image_data = base64.b64decode(base64_buffer)
            nparr = np.frombuffer(image_data, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            with self.latest_input_frame_lock:
                self.latest_input_frame = img
                ack_message = {"ack_img": "ack", "time": time}
                self.send_to_client(json.dumps(ack_message))

            if self.input_image_publisher is not None:
                self.input_image_publisher.publish_image(img)

        if "introspection" in message:
            info = message[len("introspection:") :]
            self.fps, self.lat = info.split("/")

    def update_gui(self):
        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        message = json.dumps(self.payload)
        self.send_to_client(message)

        if not self.has_received_img:
            ack_message = {"ack_img": "ack", "time": ""}
            self.send_to_client(json.dumps(ack_message))

    def payloadImage(self):
        with self.display_image_lock:
            display_image_updated = self.display_image_updated
            display_image = self.display_image

        payload = {"image": "", "shape": ""}

        if not display_image_updated:
            return payload

        shape = display_image.shape
        frame = cv2.imencode(".JPEG", display_image)[1]
        encoded_image = base64.b64encode(frame)

        payload["image"] = encoded_image.decode("utf-8")
        payload["shape"] = shape

        with self.display_image_lock:
            self.display_image_updated = False

        return payload

    def showImage(self, image):
        with self.display_image_lock:
            self.display_image = image
            self.display_image_updated = True

    def getImage(self):
        with self.latest_input_frame_lock:
            return self.latest_input_frame


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

# Redirect the console
start_console()


def showImage(image):
    gui.showImage(image)


def getImage():
    return gui.getImage()
