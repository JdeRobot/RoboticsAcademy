import json
import cv2
import base64
import threading
import time
import socket
from datetime import datetime

from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from console_interfaces.general.console import start_console


# ============================================================
# GLOBAL DEBUG CONFIG
# ============================================================
VERBOSE = True
VERBOSE_KEYS = True
VERBOSE_UDP = True
VERBOSE_GUI = True
VERBOSE_IMAGE = False
VERBOSE_LIFECYCLE = True

UDP_IP = "127.0.0.1"
UDP_PORT = 36677
WEBSOCKET_HOST = "ws://127.0.0.1:2303"

# Small pulse to emulate Classic behavior
COMMAND_PULSE_TIME = 0.25


def log(msg: str, category: str = "INFO"):
    if VERBOSE:
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"[WebGUI][{category}][{now}] {msg}")


class WebGUI(MeasuringThreadingGUI):

    def __init__(
        self,
        host: str = WEBSOCKET_HOST,
        freq: float = 30.0,
        udp_ip: str = UDP_IP,
        udp_port: int = UDP_PORT,
        pulse_time: float = COMMAND_PULSE_TIME,
    ):

        # Execution control
        self.out_period = 1.0 / freq
        self.image = None
        self.image_lock = threading.Lock()

        self.ack = True
        self.ack_lock = threading.Lock()
        self.ack_frontend = False

        self.running = True
        self.host = host
        self.msg = {"image": ""}

        # Debug / stats
        self.last_key_message = None
        self.last_udp_command = None
        self.images_sent = 0
        self.keys_received = 0
        self.last_image_shape = None

        # UDP control
        self.udp_addr = (udp_ip, udp_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_lock = threading.Lock()

        # Prevent simultaneous pulses
        self.cmd_lock = threading.Lock()

        self.sock.settimeout(0.2)

        self.pulse_time = pulse_time

        self.ideal_cycle = 80
        self.real_time_factor = 0
        self.frequency_message = {"brain": "", "gui": "", "rtf": ""}
        self.iteration_counter = 0
        self.fps = -1
        self.lat = -1

        if VERBOSE_LIFECYCLE:
            log("Initializing WebGUI", "INIT")
            log(f"WebSocket host: {self.host}", "INIT")
            log(f"UDP target: {self.udp_addr}", "INIT")
            log(f"Output frequency: {freq} Hz", "INIT")
            log(f"Pulse time: {self.pulse_time}s", "INIT")

        self.start()

        if VERBOSE_LIFECYCLE:
            log("WebGUI started", "INIT")

    # ============================================================
    # UDP SEND
    # ============================================================

    def _send_udp(self, cmd: bytes):

        try:
            with self.sock_lock:
                self.sock.sendto(cmd, self.udp_addr)

            self.last_udp_command = cmd.decode("utf-8", errors="ignore")

            if VERBOSE_UDP:
                log(
                    f"UDP command sent -> {self.last_udp_command}",
                    "UDP",
                )

        except OSError as e:
            log(f"UDP send error: {e}", "ERROR")

    # ============================================================
    # COMMAND PULSE
    # ============================================================

    def _pulse_manual_command(self, cmd: bytes):

        def pulse():

            with self.cmd_lock:

                if not self.running:
                    return

                if VERBOSE_UDP:
                    log(
                        f"Pulsed command start -> {cmd.decode()}",
                        "UDP",
                    )

                self._send_udp(cmd)

                start = time.time()

                while self.running and (time.time() - start) < self.pulse_time:
                    time.sleep(0.01)

                if self.running:
                    self._send_udp(b"US")

                if VERBOSE_UDP:
                    log("Pulsed command end -> US", "UDP")

        threading.Thread(target=pulse, daemon=True).start()

    # ============================================================
    # GUI INPUT
    # ============================================================
    def gui_in_thread(self, ws, message):

        if VERBOSE_KEYS:
            log(f"Message from frontend -> {message}", "IN")

        # ACK / start handling
        if "ack" in message:
            with self.ack_lock:
                self.ack = True
            if VERBOSE_GUI:
                log("ACK received from frontend", "GUI")
            return

        if "start" in message:
            with self.ack_lock:
                self.ack_frontend = True
            if VERBOSE_GUI:
                log("Frontend start received", "GUI")
            return

        # Track keys
        self.keys_received += 1
        self.last_key_message = message

        # --------------------------------------------------------
        # KEY DOWN EVENTS
        # --------------------------------------------------------
        if "key_w" in message:
            log("Key W pressed -> forward pulse", "KEY")
            self._pulse_manual_command(b"UVF")

        elif "key_s" in message:
            log("Key S pressed -> backward pulse", "KEY")
            self._pulse_manual_command(b"UVB")

        elif "key_a" in message:
            log("Key A pressed -> left pulse", "KEY")
            self._pulse_manual_command(b"UAL")

        elif "key_d" in message:
            log("Key D pressed -> right pulse", "KEY")
            self._pulse_manual_command(b"UAR")

        elif "key_x" in message:
            log("Key X pressed -> stop", "KEY")
            self._send_udp(b"US")

        elif "auto" in message:
            log("Auto mode requested", "KEY")
            self._send_udp(b"A")

        # --------------------------------------------------------
        # KEY UP EVENTS
        # --------------------------------------------------------
        elif "key_w_up" in message or "key_s_up" in message or "key_a_up" in message or "key_d_up" in message or "key_x_up" in message:
            log(f"Key released -> stop movement", "KEY")
            self._send_udp(b"US")

        else:
            log(f"Unknown frontend message: {message}", "WARN")
            
    # ============================================================
    # GUI OUTPUT THREAD
    # ============================================================

    def gui_out_thread(self):

        if VERBOSE_LIFECYCLE:
            log("GUI output thread started", "THREAD")

        while self.running:

            start_time = time.time()

            with self.ack_lock:
                with self.image_lock:

                    if self.ack and self.image is not None:
                        self.update_gui()
                        self.ack = False

            elapsed = time.time() - start_time
            sleep_time = max(0, self.out_period - elapsed)

            time.sleep(sleep_time)

        if VERBOSE_LIFECYCLE:
            log("GUI output thread stopped", "THREAD")

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
            log(f"update_gui error: {e}", "ERROR")

    # ============================================================
    # SET IMAGE
    # ============================================================

    def setImage(self, image):

        with self.image_lock:
            self.image = image

        if VERBOSE_IMAGE:
            shape = None if image is None else image.shape
            log(f"New image shape={shape}", "IMAGE")

    # ============================================================
    # STOP
    # ============================================================

    def stop(self):

        if VERBOSE_LIFECYCLE:
            log("Stopping WebGUI...", "STOP")

        self.running = False

        try:
            self._send_udp(b"US")
        except:
            pass

        try:
            with self.sock_lock:
                self.sock.close()
        except:
            pass

        if VERBOSE_LIFECYCLE:
            log(
                f"Final stats -> images={self.images_sent}, "
                f"keys={self.keys_received}",
                "STOP",
            )

    def __del__(self):

        try:
            self.stop()
        except:
            pass


host = WEBSOCKET_HOST
gui = WebGUI(host)

if VERBOSE_LIFECYCLE:
    log("Starting redirected console...", "MAIN")

start_console()


def showImage(img):
    gui.setImage(img)