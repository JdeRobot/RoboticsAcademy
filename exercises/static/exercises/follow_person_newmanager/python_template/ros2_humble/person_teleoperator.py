from websocket_server import WebsocketServer
import os
import socket
import sys
import time

class PersonTeleoperator:

    def __init__(self):
        self.server = None
        self.client = None
        self.host = sys.argv[1]

        self.model_address = ("127.0.0.1", 36677)
        self.model_client = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

    # Function to read the message from websocket
    def get_message(self, client, server, message):
        print("\n\n RECEIVED MSG IN TLEOPERATOR!!\n\n")
        if (message[:7] == "#teleop"):
            mode = message[message.find('_')+1:]
            print("\n\nTeleoperate mode: " + str(mode) + "\n\n")
            if mode == "true":
                self.model_client.sendto(str.encode("US-"), self.model_address) # User Stop
            elif mode == "false":
                self.model_client.sendto(str.encode("A--"), self.model_address) # Autonomous

        elif (message[:4] == "#key"):
            mode = message[message.find('_')+1:]
            print("\nKey " + str(mode) +"\n")
            if mode == "w":
                self.model_client.sendto(str.encode("UVF"), self.model_address) # User Velocity Forward
            elif mode == "s":
                self.model_client.sendto(str.encode("UVB"), self.model_address) # User Velocity Backward
            elif mode == "a":
                self.model_client.sendto(str.encode("UAL"), self.model_address) # User Angular Left
            elif mode == "d":
                self.model_client.sendto(str.encode("UAR"), self.model_address) # User Angular Right
            elif mode == "x":
                self.model_client.sendto(str.encode("US-"), self.model_address) # User Stop model


    # Function that gets called when the connected closes
    def handle_close(self, client, server):
        print("\n\n CLOSED TOELEOPERWATOR CLIENTTTTT!!\n\n")
        self.client = None
        self.server.allow_new_connections()
        print(client, 'closed')

    # Called when a new client is received
    def get_client(self, client, server):
        self.client = client
        self.server.deny_new_connections()
        print("\n\n CONNECTED TOELEOPERWATOR CLIENTTTTT!!\n\n")
        print(client, 'connected')


    # Activate the server
    def run_server(self):
        self.server = WebsocketServer(port=7164, host=self.host)
        self.server.set_fn_new_client(self.get_client)
        self.server.set_fn_message_received(self.get_message)
        self.server.set_fn_client_left(self.handle_close)

        home_dir = os.path.expanduser('~')

        logged = False
        while not logged:
            try:
                f = open(f"{home_dir}/ws_teleop.log", "w")
                f.write("websocket_teleop=ready")
                f.close()
                logged = True
            except:
                print("~/ws_teleop.log could not be opened for write", flush=True)
                time.sleep(0.1)

        self.server.run_forever()


# Execute!
if __name__ == "__main__":
    server = PersonTeleoperator()
    server.run_server()