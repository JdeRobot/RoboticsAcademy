import json
import cv2
import base64
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32MultiArray, Empty
from cv_bridge import CvBridge

from gui_interfaces.general.measuring_threading_gui_harmonic import MeasuringThreadingGUI
from console_interfaces.general.console import start_console


class WebGUINode(Node):
    def __init__(self, gui_instance):
        super().__init__("webgui_3d_reconstruction")
        self.gui = gui_instance
        self.bridge = CvBridge()
        
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.create_subscription(Image, '/webgui/image_left', self.img_left_cb, qos_profile)
        self.create_subscription(Image, '/webgui/image_right', self.img_right_cb, qos_profile)
        self.create_subscription(Bool, '/webgui/paint_matching', self.paint_match_cb, 10)
        self.create_subscription(String, '/webgui/points_new', self.pts_new_cb, 10)
        self.create_subscription(String, '/webgui/points_all', self.pts_all_cb, 10)
        self.create_subscription(Float32MultiArray, '/webgui/image_matching', self.img_match_cb, 10)
        self.create_subscription(Empty, '/webgui/clear_points', self.clear_cb, 10)

    def img_left_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.gui.image_show_lock:
                self.gui.image1_to_be_shown = cv_image
                self.gui.image_to_be_shown_updated = True
        except Exception:
            pass

    def img_right_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.gui.image_show_lock:
                self.gui.image2_to_be_shown = cv_image
                self.gui.image_to_be_shown_updated = True
        except Exception:
            pass

    def paint_match_cb(self, msg):
        self.gui.paint_matching = "T" if msg.data else "F"

    def pts_new_cb(self, msg):
        try:
            pts = json.loads(msg.data)
            self.gui.ShowNewPoints(pts)
        except Exception:
            pass

    def pts_all_cb(self, msg):
        try:
            pts = json.loads(msg.data)
            self.gui.ShowAllPoints(pts)
        except Exception:
            pass

    def img_match_cb(self, msg):
        if len(msg.data) >= 4:
            self.gui.showImageMatching(msg.data[0], msg.data[1], msg.data[2], msg.data[3])

    def clear_cb(self, msg):
        self.gui.ClearAllPoints()


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):
        super().__init__(host)

        if not rclpy.ok():
            rclpy.init()

        self.image1_to_be_shown = None
        self.image2_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.point_to_save = []
        self.point_to_send = []
        self.point_set = set()

        self.matching = []
        self.paint_matching = "F"
        self.needs_clear = False

        self.payload = {"img1": "", "img2": "", "pts": "", "match": "", "p_match": "F"}

        self.ros_node = WebGUINode(self)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.ros_node)

        self.start()

    def update_gui(self):
        self.executor.spin_once(timeout_sec=0)
        
        if getattr(self, 'needs_clear', False):
            self.send_to_client("#res")
            self.needs_clear = False
            return
            
        payload1, payload2 = self.payloadImage()
        self.payload["img1"] = json.dumps(payload1)
        self.payload["img2"] = json.dumps(payload2)
        
        length_point_send = len(self.point_to_send)
        if length_point_send != 0:
            if length_point_send > 500:
                self.payload["pts"] = json.dumps(self.point_to_send[0:500])
                del self.point_to_send[0:500]
            else:
                self.payload["pts"] = json.dumps(self.point_to_send)
                del self.point_to_send[0:length_point_send]
        else:
            self.payload["pts"] = json.dumps([])
            
        self.payload["match"] = json.dumps(self.matching)
        self.matching = []
        
        self.payload["p_match"] = self.paint_matching

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image1_to_be_shown = self.image1_to_be_shown
            image2_to_be_shown = self.image2_to_be_shown

        payload1 = {"img": ""}
        payload2 = {"img": ""}

        if not image_to_be_shown_updated:
            return payload1, payload2

        if image1_to_be_shown is not None:
            image1 = cv2.resize(image1_to_be_shown, (0, 0), fx=0.50, fy=0.50)
            frame1 = cv2.imencode(".JPEG", image1)[1]
            payload1["img"] = base64.b64encode(frame1).decode("utf-8")

        if image2_to_be_shown is not None:
            image2 = cv2.resize(image2_to_be_shown, (0, 0), fx=0.50, fy=0.50)
            frame2 = cv2.imencode(".JPEG", image2)[1]
            payload2["img"] = base64.b64encode(frame2).decode("utf-8")

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload1, payload2

    def showImages(self, image1, image2, paint_matching):
        self.paint_matching = "T" if paint_matching else "F"
        if not np.array_equal(self.image1_to_be_shown, image1) or not np.array_equal(self.image2_to_be_shown, image2):
            with self.image_show_lock:
                self.image1_to_be_shown = image1
                self.image2_to_be_shown = image2
                self.image_to_be_shown_updated = True

    def ShowNewPoints(self, points):
        for p in points:
            tp = tuple(p)
            if tp not in self.point_set:
                self.point_set.add(tp)
                self.point_to_save.append(p)
                self.point_to_send.append(p)

    def ShowAllPoints(self, points):
        self.ClearAllPoints()
        for p in points:
            tp = tuple(p)
            self.point_set.add(tp)
            self.point_to_save.append(p)
            self.point_to_send.append(p)

    def showImageMatching(self, x1, y1, x2, y2):
        self.matching.append([x1, y1, x2, y2])

    def ClearAllPoints(self):
        self.point_to_save.clear()
        self.point_to_send.clear()
        self.matching.clear()
        self.point_set.clear()
        self.needs_clear = True

    def reset_gui(self):
        self.ClearAllPoints()


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

start_console()

def showImages(image1, image2, paint_matching):
    gui.showImages(image1, image2, paint_matching)

def ShowNewPoints(points):
    gui.ShowNewPoints(points)

def ShowAllPoints(points):
    gui.ShowAllPoints(points)

def showImageMatching(x1, y1, x2, y2):
    gui.showImageMatching(x1, y1, x2, y2)

def ClearAllPoints():
    gui.ClearAllPoints()