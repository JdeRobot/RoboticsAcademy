import json
import logging
from queue import Queue
from uuid import uuid4
from websocket_server import WebsocketServer

from src.comms.consumer_message import ManagerConsumerMessageException, ManagerConsumerMessage
from src.logging.log_manager import LogManager


class Client:
    def __init__(self, **kwargs):
        self.id = kwargs['id']
        self.handler = kwargs['handler']
        self.address = kwargs['address']


class ManagerConsumer:
    """
    Websocket server consumer for new Robotics Application Manager aka. RAM
    Supports single client connection to RAM
    TODO: Better handling of single client connections, closing and redirecting
    """

    def __init__(self, host, port, manager_queue: Queue):
        self.host = host
        self.port = port
        self.server = WebsocketServer(host=host, port=port, loglevel=logging.INFO)
        self.server.set_fn_new_client(self.handle_client_new)
        self.server.set_fn_client_left(self.handle_client_disconnect)
        self.server.set_fn_message_received(self.handle_message_received)
        self.client = None
        self.manager_queue = manager_queue

    def handle_client_new(self, client, server):
        LogManager.logger.info(f"client connected: {client}")
        self.client = client
        self.server.deny_new_connections()

    def handle_client_disconnect(self, client, server):
        if client is None:
            return

        LogManager.logger.info(f"client disconnected: {client}")
        message = ManagerConsumerMessage(**{'id': str(uuid4()), 'command': 'reset'})
        self.manager_queue.put(message)
        self.client = None
        self.server.allow_new_connections()

    def handle_message_received(self, client, server, websocket_message):
        LogManager.logger.info(f"message received: {websocket_message} from client {client}")
        message = None
        try:
            s = json.loads(websocket_message)
            message = ManagerConsumerMessage(**s)
            self.manager_queue.put(message)
        except Exception as e:
            if message is not None:
                ex = ManagerConsumerMessageException(id=message.id, message=str(e))
            else:
                ex = ManagerConsumerMessageException(id=str(uuid4()), message=str(e))
            self.server.send_message(client, str(ex))
            raise e

    def send_message(self, message_data, command=None):
        if self.client is not None and self.server is not None:
            if isinstance(message_data, ManagerConsumerMessage):
                message = message_data
            elif isinstance(message_data, ManagerConsumerMessageException):
                message = message_data.consumer_message()
            else:
                message = ManagerConsumerMessage(id=str(uuid4()), command=command, data=message_data)

            self.server.send_message(self.client, str(message))

    def start(self):
        self.server.run_forever(threaded=True)

    def stop(self):
        self.server.shutdown_gracefully()
