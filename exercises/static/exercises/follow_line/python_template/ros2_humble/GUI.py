import cv2
import base64
import json
import threading
import rclpy
from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from console_interfaces.general.console import start_console

from hal_interfaces.general.camera import CameraNode
from cv_bridge import CvBridge
from hal_interfaces.general.odometry import OdometryNode
from lap import Lap
from sensor_msgs.msg import Image
from rclpy.node import Node

import sys

sys.path.insert(0, '/RoboticsApplicationManager')

from manager.ram_logging.log_manager import LogManager

class WebGUIImagePublisher(Node):
    """Internal publisher to create /webgui_image topic"""
    def __init__(self):
        super().__init__('webgui_image_publisher_internal')
        self.publisher = self.create_publisher(Image, '/webgui_image', 10)

class GUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        if not rclpy.ok():
            rclpy.init()
        
        self.webgui_publisher = WebGUIImagePublisher()
        self.camera_node = None
        self.auto_image_mode = False
        self._setup_auto_mode()

        self.payload = {'image': '', 'lap': '', 'map': ''}
        self.bridge = CvBridge()

        self.pose3d_object = OdometryNode("/odom")

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.webgui_publisher)
        if self.camera_node:
            self.executor.add_node(self.camera_node)
        self.executor.add_node(self.pose3d_object)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        self.lap = Lap(self.pose3d_object)

        if self.auto_image_mode:
            self.auto_image_thread = threading.Thread(target=self._unified_image_loop, daemon=True)
            self.auto_image_thread.start()

        self.start()

    def _setup_auto_mode(self):
        """Set up automatic image subscription"""
        try:
            temp_node = rclpy.create_node('topic_checker_temp')
            topic_names_and_types = temp_node.get_topic_names_and_types()
            topic_names = [topic_name for topic_name, _ in topic_names_and_types]
            
            if '/webgui_image' in topic_names:
                self.camera_node = CameraNode("/webgui_image")
                self.auto_image_mode = True
            
            temp_node.destroy_node()
            
        except Exception:
            pass

    def _unified_image_loop(self):
        """Unified image handling loop"""
        while True:
            try:
                if self.camera_node:
                    image = self.camera_node.getImage()
                    if image is not None:
                        self.showImage(image.data)
                        
                threading.Event().wait(0.033)  # ~30 FPS
            except Exception:
                threading.Event().wait(1.0)

    def gui_in_thread(self, ws, message):
        if "ack" in message:
            with self.ack_lock:
                self.ack = True
                self.ack_frontend = True
        elif "start" in message:
            self.lap.unpause()
        elif "pause" in message:
            self.lap.pause()

    def update_gui(self):
        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)
        
        lapped = self.lap.check_threshold()
        self.payload["lap"] = ""
        if lapped is not None:
            self.payload["lap"] = str(lapped)
            
        pose = self.pose3d_object.getPose3d()
        pos_message = str((pose.x, pose.y))
        self.payload["map"] = pos_message
        
        message = json.dumps(self.payload)
        self.send_to_client(message)

    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image_to_be_shown = self.image_to_be_shown

        payload = {'image': '', 'shape': ''}

        if not image_to_be_shown_updated or image_to_be_shown is None:
            return payload

        shape = image_to_be_shown.shape
        frame = cv2.imencode('.JPEG', image_to_be_shown)[1]
        encoded_image = base64.b64encode(frame)

        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload

    def showImage(self, image):
        """Single point of entry for all images"""
        with self.image_show_lock:
            self.image_to_be_shown = image
            self.image_to_be_shown_updated = True

    def get_image_mode(self):
        return {
            'auto_mode': self.auto_image_mode,
            'topic_subscribed': '/webgui_image' if self.auto_image_mode else None,
            'manual_mode_available': True
        }

_gui = None
_gui_lock = threading.Lock()

def get_gui():
    global _gui
    with _gui_lock:
        if _gui is None:
            host = "ws://127.0.0.1:2303"
            _gui = GUI(host)
            start_console()
    return _gui

def showImage(image):
    """Display an image in the GUI"""
    gui = get_gui()
    if gui is not None:
        gui.showImage(image)

def get_image_mode():
    gui = get_gui()
    if gui is not None:
        return gui.get_image_mode()
    return {'auto_mode': False, 'topic_subscribed': None, 'manual_mode_available': True}