import json
import cv2
import base64
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor

from gui_interfaces.general.measuring_threading_gui_harmonic import MeasuringThreadingGUI
from console_interfaces.general.console import start_console

class WebGUINode(Node):
    def __init__(self, gui_instance):
        super().__init__("webgui_car_junction")
        self.gui = gui_instance
        self.bridge = CvBridge()
        
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Image, '/webgui/image', self.img_cb, qos_profile)

    def img_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.gui.set_incoming_image(cv_image)
        except Exception:
            pass

class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        if not rclpy.ok():
            rclpy.init()

        self.incoming_image = None
        self.incoming_lock = threading.Lock()
        
        self.encoded_payload = ""
        self.payload_lock = threading.Lock()
        self.payload = {"image": ""}

        self.ros_node = WebGUINode(self)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.ros_node)

        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        self.unified_thread = threading.Thread(target=self._unified_image_loop, daemon=True)
        self.unified_thread.start()

        self.start()

    def set_incoming_image(self, image):
        with self.incoming_lock:
            self.incoming_image = image

    def _unified_image_loop(self):
        while True:
            try:
                with self.incoming_lock:
                    image = self.incoming_image
                    self.incoming_image = None

                if image is not None:
                    h, w = image.shape[:2]
                    if w > 640 or h > 480:
                        image = cv2.resize(image, (640, 480))

                    _, frame = cv2.imencode(".JPEG", image, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                    encoded = base64.b64encode(frame).decode("utf-8")
                    
                    with self.payload_lock:
                        self.encoded_payload = encoded

                threading.Event().wait(0.033)
            except Exception:
                threading.Event().wait(1.0)

    def update_gui(self):
        with self.payload_lock:
            if not self.encoded_payload:
                return
            current_payload = self.encoded_payload
            self.encoded_payload = ""

        self.payload["image"] = json.dumps({"image": current_payload, "shape": [480, 640, 3]})
        message = json.dumps(self.payload)
        self.send_to_client(message)

    def setImage(self, image):
        self.set_incoming_image(image)

host = "ws://127.0.0.1:2303"
gui = WebGUI(host)
start_console()

def showImage(image):
    gui.setImage(image)