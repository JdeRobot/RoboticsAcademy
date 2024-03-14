import json
import cv2
import base64
import threading
import time
import websocket


class UnifiedGUI:
    def __init__(self, host="ws://127.0.0.1:2303", ideal_cycle=80):
        self.host = host
        self.ideal_cycle = ideal_cycle
        self.image_lock = threading.Lock()
        self.acknowledge_lock = threading.Lock()
        self.acknowledge = False
        self.image = None
        self.msg = {"image": ""}
        self.running = True
        self.client = None

        # Initialize and start the WebSocket client thread
        threading.Thread(target=self.run_websocket, daemon=True).start()

        # Initialize and start the image sending thread (GUI output thread)
        threading.Thread(
            target=self.gui_out_thread, name="gui_out_thread", daemon=True
        ).start()

    def run_websocket(self):
        """Runs the WebSocket client."""
        self.client = websocket.WebSocketApp(self.host, on_message=self.on_message)
        self.client.run_forever(ping_timeout=None, ping_interval=0)

    def on_message(self, ws, message):
        """Handles incoming WebSocket messages."""
        if message.startswith("#ack"):
            with self.acknowledge_lock:
                self.acknowledge = True

    def showImage(self, image):
        """Sets the image to be shown."""
        with self.image_lock:
            self.image = image

    def gui_out_thread(self):
        """Sends the image to the server at a fixed interval."""
        while self.running:
            start_time = time.time()
            self.send_image()
            elapsed = time.time() - start_time
            sleep_time = max(0, (self.ideal_cycle / 1000.0) - elapsed)
            time.sleep(sleep_time)

    def send_image(self):
        """Prepares and sends the current image to the WebSocket server."""
        with self.image_lock:
            if self.image is not None:
                _, encoded_image = cv2.imencode(".JPEG", self.image)
                payload = {
                    "image": base64.b64encode(encoded_image).decode("utf-8"),
                    "shape": self.image.shape,
                }
                self.msg["image"] = json.dumps(payload)
                message = json.dumps(self.msg)
                try:
                    if self.client:
                        self.client.send(message)
                except Exception as e:
                    print(f"Error sending message: {e}")


host = "ws://127.0.0.1:2303"
gui = UnifiedGUI(host)


def showImage(img):
    gui.showImage(img)
