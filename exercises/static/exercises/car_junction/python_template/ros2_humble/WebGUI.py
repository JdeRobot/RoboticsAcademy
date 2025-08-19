import json
import cv2
import base64
import threading
import rclpy
import numpy as np
from gui_interfaces.general.measuring_threading_gui_harmonic import MeasuringThreadingGUI
from console_interfaces.general.console import start_console
from HAL import getFrontCameraData, getPose3d


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)
    
        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()
        self.payload = {"image": "", "map": ""}
       
        self.camera_thread = threading.Thread(target=self._camera_update_loop, daemon=True)
        self.camera_thread.start()

    def _camera_update_loop(self):
        while True:
            image_raw=getFrontCameraData()
            try:
                if image_raw and hasattr(image_raw, 'data'): 
                    image_data = image_raw.data 
                if image_data is not None:
                    self.showImage(image_data) 
                threading.Event().wait(0.033)  # ~30 FPS
            except Exception:
                threading.Event().wait(1.0)
 
    def update_gui(self):
        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        pose = getPose3d()
        pos_message = str((pose.x, pose.y))
        self.payload["map"] = pos_message

        message = json.dumps(self.payload)
        self.send_to_client(message)
    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image_to_be_shown = self.image_to_be_shown

        payload = {"image": "", "shape": ""}

        if not image_to_be_shown_updated or image_to_be_shown is None:
            return payload

        shape = image_to_be_shown.shape
        frame = cv2.imencode(".JPEG", image_to_be_shown)[1]
        encoded_image = base64.b64encode(frame)

        payload["image"] = encoded_image.decode("utf-8")
        payload["shape"] = shape

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload

    def showImage(self, image):
        """Single point of entry for all images"""
        with self.image_show_lock:
            self.image_to_be_shown = image
            self.image_to_be_shown_updated = True

    def reset_gui(self):
        """Reset GUI state"""
        with self.frame_lock:
            self.current_frame = None
        self.payload = {"image": "", "map": ""}

host = "ws://127.0.0.1:2303"
gui = WebGUI(host)
start_console()

def showImage(image):
    """Display an image in the GUI"""
    if gui is not None:
        gui.showImage(image)

def get_gui():
    """Backward compatibility function"""
    return gui