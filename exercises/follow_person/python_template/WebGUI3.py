import json
import cv2
import base64
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # Para publicar velocidades lineales y angulares
from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from console_interfaces.general.console import start_console


# ============================================================
# CONFIG
# ============================================================
WEBSOCKET_HOST = "ws://127.0.0.1:2303"
FPS = 30

LINEAR_SPEED = 0.01
ANGULAR_SPEED = 0.01


# ============================================================
# LOG
# ============================================================
def log(msg):
    now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[WebGUI][{now}] {msg}")


# ============================================================
# WebGUI
# ============================================================
class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host=WEBSOCKET_HOST, freq=FPS):

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
        self.images_sent = 0
        self.keys_received = 0
        self.last_image_shape = None

        # ----------------------------
        # ROS2 Node y Publisher
        # ----------------------------
        rclpy.init()
        self.node = Node("webgui_person_control")
        self.pub = self.node.create_publisher(Twist, "/person/cmd_vel", 10)

        self.start()
        log("WebGUI iniciado")

    # ============================================================
    # INPUT TECLADO
    # ============================================================
    def gui_in_thread(self, ws, message):
        """Recibe mensajes del frontend y publica velocidades según tecla."""

        # ACK del frontend
        if "ack" in message:
            with self.ack_lock:
                self.ack = True
            return

        if "start" in message:
            return

        # Log teclas
        self.keys_received += 1
        # log(f"Tecla: {message}")

        twist = Twist()

        # =====================
        # LINEAR
        # =====================
        if message == "key_w":
            twist.linear.x = LINEAR_SPEED
        elif message == "key_s":
            twist.linear.x = -LINEAR_SPEED
        elif message in ["key_w_up", "key_s_up"]:
            twist.linear.x = 0.0

        # =====================
        # ANGULAR
        # =====================
        if message == "key_a":
            twist.angular.z = ANGULAR_SPEED
        elif message == "key_d":
            twist.angular.z = -ANGULAR_SPEED
        elif message in ["key_a_up", "key_d_up"]:
            twist.angular.z = 0.0

        # Publicar en el topic
        self.pub.publish(twist)

    # ============================================================
    # OUTPUT (ENVÍO IMAGEN)
    # ============================================================
    def gui_out_thread(self):
        while self.running:
            start = time.time()
            with self.ack_lock:
                with self.image_lock:
                    if self.ack and self.image is not None:
                        self.update_gui()
                        self.ack = False
            elapsed = time.time() - start
            time.sleep(max(0, self.out_period - elapsed))

    # ============================================================
    # SEND IMAGE
    # ============================================================
    def update_gui(self):
        try:
            if self.image is None:
                return

            _, encoded = cv2.imencode(".JPEG", self.image)
            payload = {
                "image": base64.b64encode(encoded).decode(),
                "shape": self.image.shape,
            }

            self.msg["image"] = json.dumps(payload)
            self.send_to_client(json.dumps(self.msg))
            self.images_sent += 1
            self.last_image_shape = self.image.shape

        except Exception as e:
            log(f"update_gui error: {e}")

    # ============================================================
    # SET IMAGE
    # ============================================================
    def setImage(self, image):
        with self.image_lock:
            self.image = image

    # ============================================================
    # STOP
    # ============================================================
    def stop(self):
        self.running = False


# ==========================
# Inicializar GUI
# ==========================
host = WEBSOCKET_HOST
gui = WebGUI(host)
start_console()


# Función para setear la imagen
def showImage(img):
    gui.setImage(img)
