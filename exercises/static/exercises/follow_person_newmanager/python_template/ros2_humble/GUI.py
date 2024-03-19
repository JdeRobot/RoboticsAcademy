import json
import cv2
import base64
import threading
import time
import websocket
from src.manager.ram_logging.log_manager import LogManager
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose
import rclpy


class ThreadGUI:
    def __init__(self, host="ws://127.0.0.1:2303", ideal_cycle=30):
        self.host = host
        self.ideal_cycle = ideal_cycle
        self.image_lock = threading.Lock()
        self.acknowledge_lock = threading.Lock()
        self.acknowledge = False
        self.image = None
        self.msg = {"image": ""}
        self.running = True
        self.node = rclpy.create_node("person_mover")
        self.service_client = None
        self.person_pose = Pose()

        # Initialize and start the WebSocket client thread
        threading.Thread(target=self.run_websocket, daemon=True).start()

        # Initialize and start the image sending thread (GUI output thread)
        threading.Thread(
            target=self.gui_out_thread, name="gui_out_thread", daemon=True
        ).start()

        # Initialize the service client
        self.service_client = self.node.create_client(
            SetEntityState, "/follow_person/set_entity_state"
        )
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        self.request = SetEntityState.Request()

    def run_websocket(self):
        self.client = websocket.WebSocketApp(self.host, on_message=self.gui_in_thread)
        self.client.run_forever(ping_timeout=None, ping_interval=0)

    def move_object(self):
        self.request.state.name = "PersonToFollow"
        self.request.state.pose = self.person_pose
        self.request.state.reference_frame = "world"
        future = self.service_client.call_async(self.request)
        rclpy.spin_until_future_complete(self.node, future)

    def gui_in_thread(self, ws, message):
        if message.startswith("#ack"):
            with self.acknowledge_lock:
                self.acknowledge = True
        else:
            self.person_pose.position.x += 1
            self.move_object()
            LogManager.logger.info(f"Message {message}")

    def showImage(self, image):
        with self.image_lock:
            self.image = image

    def gui_out_thread(self):
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
                    LogManager.logger.info(f"Error sending message: {e}")


host = "ws://127.0.0.1:2303"
gui = ThreadGUI(host)


# Expose the gui showImage function
def showImage(img):
    gui.showImage(img)
