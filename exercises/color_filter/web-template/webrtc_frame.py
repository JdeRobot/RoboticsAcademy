import threading
import cv2
import numpy as np
from websocket_server import WebsocketServer
from gui import GUI

# WebRTC Frame Class
class WebrtcFrame:
    # Initialization function
    # The actual initialization
    def __init__(self, host):
        t = threading.Thread(target=self.run_server)

        self.reload = False

        self.server = None
        self.client = None

        self.host = host
        self.frame_rgb = None
        t.start()

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initWeb(cls, host):
        new_instance = cls(host)
        return new_instance

    # Gets called when there is an incoming message from the client
    def handle(self, client, server, message):

        if(message == "#pong"):
            self.server.send_message(self.client, "#ping")
            return

        try:
            # Once received turn the reload flag up and send it to execute_thread function
            message_array=np.fromstring(message, dtype=int, sep=',')
            message_array.resize(240,320,3)
            frame_bgr = np.uint8(message_array)
            self.frame_rgb = cv2.cvtColor(frame_bgr ,cv2.COLOR_BGR2RGB)
            self.reload = True

            #self.execute_thread(frame)
        except:
            pass

    # Function that gets called when the connected closes
    def getImage(self):
        return self.frame_rgb

    def connected(self, client, server):
        self.client = client

        # Initialize the ping message
        self.server.send_message(self.client, "#ping")

        print(client, 'connected')

    # Function that gets called when the connected closes
    def handle_close(self, client, server):
        print(client, 'closed')

    def run_server(self):
        self.server = WebsocketServer(port=1831, host=self.host)
        self.server.set_fn_new_client(self.connected)
        self.server.set_fn_client_left(self.handle_close)
        self.server.set_fn_message_received(self.handle)
        self.server.run_forever()
