import cv2
import base64
import json
import threading
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from console_interfaces.general.console import start_console
from lap import Lap

sys.path.insert(0, "/RoboticsApplicationManager")
from robotics_application_manager import LogManager


class WebGUINode(Node):
    def __init__(self, gui_instance):
        super().__init__("webgui_end_to_end")
        self.gui = gui_instance
        self.bridge = CvBridge()
        
        self.create_subscription(Image, '/webgui/image', self.img_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

    def img_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.gui.image_show_lock:
                self.gui.image_to_be_shown = cv_image
                self.gui.image_to_be_shown_updated = True
        except Exception:
            pass
            
    def odom_cb(self, msg):
        # Actualizamos las variables de la GUI directamente
        self.gui.current_x = msg.pose.pose.position.x
        self.gui.current_y = msg.pose.pose.position.y


# Envoltorio para mantener lap.py original intacto
class Pose3DMock:
    def __init__(self, gui):
        self.gui = gui
        
    def getPose3d(self):
        # lap.py espera un objeto con atributos x e y
        class Pose:
            pass
        p = Pose()
        p.x = self.gui.current_x
        p.y = self.gui.current_y
        return p


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        if not rclpy.ok():
            rclpy.init()

        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()
        
        self.current_x = 0.0
        self.current_y = 0.0

        self.payload = {"image": "", "lap": "", "map": ""}
        
        self.ros_node = WebGUINode(self)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.ros_node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        # Instanciamos nuestro Mock y se lo pasamos al Lap original
        self.pose3d_mock = Pose3DMock(self)
        self.lap = Lap(self.pose3d_mock)
        
        self.start()

    def gui_in_thread(self, ws, message):
        if "ack" in message:
            with self.ack_lock:
                self.ack = True
        elif "start" in message:
            with self.ack_lock:
                self.ack_frontend = True
        elif "startLap" in message:
            self.lap.unpause()
        elif "pause" in message:
            self.lap.pause()
        else:
            LogManager.logger.error("Unsupported msg")

    def update_gui(self):
        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        lapped = self.lap.check_threshold()
        self.payload["lap"] = ""
        if lapped is not None:
            self.payload["lap"] = str(lapped)

        pos_message = str((self.current_x, self.current_y))
        self.payload["map"] = pos_message

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image_to_be_shown = self.image_to_be_shown

        payload = {"image": "", "shape": ""}

        if not image_to_be_shown_updated:
            return payload

        if image_to_be_shown is not None:
            shape = image_to_be_shown.shape
            frame = cv2.imencode(".JPEG", image_to_be_shown)[1]
            encoded_image = base64.b64encode(frame)

            payload["image"] = encoded_image.decode("utf-8")
            payload["shape"] = shape

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload

    def showImage(self, image):
        with self.image_show_lock:
            self.image_to_be_shown = image
            self.image_to_be_shown_updated = True


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)
start_console()

def showImage(image):
    gui.showImage(image)