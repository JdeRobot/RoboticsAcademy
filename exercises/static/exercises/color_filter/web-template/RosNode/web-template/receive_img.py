
import cv2
import base64
import threading
import websocket
import numpy as np


class ReceiveImage():

    def __init__(self,host):
        t = threading.Thread(target=self.run_server)
        self.server = None
        self.client = None
        self.image = None
        self.host = host
        self.ws = None

        t.start()

    @classmethod
    def initGUI(cls, host):
        new_instance = cls(host)
        return new_instance

    def on_message(self, message):
        img_str = base64.b64decode(message)
        nparr = np.frombuffer(img_str, np.uint8)
        if len(nparr) > 50:
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            self.image = img


    def on_error(self, error):
        print(error)

    def on_close(self):
        print("### closed ###")

    def getImage(self):
        return self.image
    # Activate the server
    def run_server(self):
        ws = websocket.WebSocketApp("ws://0.0.0.0:60002",
                                    on_message=self.on_message,
                                    on_error=self.on_error,
                                    on_close=self.on_close)
        self.ws = ws
        self.ws.run_forever(ping_interval=15)


if __name__ == '__main__':
    ReceiveImage("0.0.0.0")