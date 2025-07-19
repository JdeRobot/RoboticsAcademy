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

import sys

sys.path.insert(0, '/RoboticsApplicationManager')

from manager.ram_logging.log_manager import LogManager

class GUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        # Always tries to set up camera node for auto mode
        self.camera_node = None
        self.auto_image_mode = False
        self._setup_auto_mode_if_possible()

        # Payload vars
        self.payload = {'image': '', 'lap': '', 'map': ''}
        self.bridge = CvBridge()

        self.pose3d_object = OdometryNode("/odom")

        # ROS2 Executor
        self.executor = rclpy.executors.MultiThreadedExecutor()
        if self.camera_node:
            self.executor.add_node(self.camera_node)
        self.executor.add_node(self.pose3d_object)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        self.lap = Lap(self.pose3d_object)

        # Start the unified image handling thread
        if self.auto_image_mode:
            self.auto_image_thread = threading.Thread(target=self._unified_image_loop, daemon=True)
            self.auto_image_thread.start()
            LogManager.logger.info("Started auto image mode - will display images from /webgui_image topic")
        else:
            LogManager.logger.info("Manual image mode - use showImage() to display images")

        self.start()

    def _setup_auto_mode_if_possible(self):
        """Try to set up automatic image subscription if topic exists"""
        try:
            # Wait a moment for topics to be available
            import time
            time.sleep(1)
            
            if not rclpy.ok():
                rclpy.init()
            
            temp_node = rclpy.create_node('topic_checker_temp')
            
            # Check multiple times in case topic appears later
            for attempt in range(3):
                topic_names_and_types = temp_node.get_topic_names_and_types()
                topic_names = [topic_name for topic_name, _ in topic_names_and_types]
                print(topic_names)
                
                if '/webgui_image' in topic_names:
                    LogManager.logger.info("Found /webgui_image topic! Setting up automatic mode.")
                    self.camera_node = CameraNode("/webgui_image")
                    self.auto_image_mode = True
                    temp_node.destroy_node()
                    return
                    
                time.sleep(0.5)  # Wait between attempts
            
            temp_node.destroy_node()
            LogManager.logger.info("No /webgui_image topic found. Manual mode only.")
            
        except Exception as e:
            LogManager.logger.warning(f"Error setting up auto mode: {e}. Manual mode only.")

    def _unified_image_loop(self):
        """
        Unified image handling - gets images from camera and calls showImage()
        """
        LogManager.logger.info("Auto image loop started")
        
        while True:
            try:
                if self.camera_node:
                    image = self.camera_node.getImage()
                    if image is not None:
                        # Use the SAME showImage path as manual mode!
                        self.showImage(image.data)
                        
                threading.Event().wait(0.033)  # ~30 FPS
            except Exception as e:
                LogManager.logger.error(f"Error in auto image loop: {e}")
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
        else:
            LogManager.logger.error("Unsupported msg")

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

        if not image_to_be_shown_updated:
            return payload

        if image_to_be_shown is None:
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
        #Single point of entry for ALL images - whether from auto mode or manual calls
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
    #Display an image in the GUI. Works in both auto and manual modes!
    gui = get_gui()
    if gui is not None:
        gui.showImage(image)

def get_image_mode():
    #Get the current image mode of the GUI.
    gui = get_gui()
    if gui is not None:
        return gui.get_image_mode()
    return {'auto_mode': False, 'topic_subscribed': None, 'manual_mode_available': True}