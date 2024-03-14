import json
import cv2
import base64
from multiprocessing import Process, Manager, Value
import time
import websocket
import threading


def websocket_thread(shared_data, host):
    """This function runs in its own thread to handle WebSocket events."""

    def on_message(ws, message):
        """Handles incoming WebSocket messages."""
        print(f"Received message: {message}")
        if message.startswith("#ack"):
            shared_data["acknowledge"] = True

    def on_error(ws, error):
        print(f"WebSocket error: {error}")

    def on_close(ws, close_status_code, close_msg):
        print("WebSocket closed")

    def on_open(ws):
        print("WebSocket connection opened.")
        while not ws.sock.connected:
            time.sleep(1)  # Wait for connection before proceeding
        while True:
            if shared_data["new_image"]:
                send_image(ws, shared_data["image"])
                shared_data["new_image"] = False
            time.sleep(0.1)  # Prevent this loop from consuming too much CPU

    def send_image(ws, image):
        """Encodes and sends the image via WebSocket."""
        _, encoded_image = cv2.imencode(".JPEG", image)
        payload = {
            "image": base64.b64encode(encoded_image).decode("utf-8"),
            "shape": image.shape,
        }
        message = json.dumps({"image": json.dumps(payload)})
        ws.send(message)

    ws = websocket.WebSocketApp(
        host, on_message=on_message, on_error=on_error, on_close=on_close
    )
    ws.on_open = on_open
    ws.run_forever()


class UnifiedGUI:
    def __init__(self, host="ws://127.0.0.1:2303"):
        self.host = host
        manager = Manager()
        self.shared_data = manager.dict(image=None, new_image=False, acknowledge=False)

        # Start the WebSocket communication in a separate process
        self.comm_process = Process(
            target=self.run_comm_process,
            args=(self.shared_data, self.host),
            daemon=True,
        )
        self.comm_process.start()

    def run_comm_process(self, shared_data, host):
        """Runs the communication process."""
        websocket_thread(shared_data, host)

    def showImage(self, image):
        """Sets the image to be shown in shared memory."""
        self.shared_data["image"] = image
        self.shared_data["new_image"] = True


# Example usage:
host = "ws://127.0.0.1:2303"
gui = UnifiedGUI(host)
