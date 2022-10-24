from __future__ import annotations
import asyncio
import json
from uuid import uuid4

import websockets
from websockets.server import WebSocketServerProtocol

from src.comms.consumer_message import ManagerConsumerMessage, ManagerConsumerMessageException


class ManagerConsumer:
    """
    Robotics Academy websocket consumer
    """

    def __init__(self, host, port):
        from src.manager.manager import Manager


        """
        Initializes a new ManagerConsumer
        @param host: host for connections, '0.0.0.0' to bind all interfaces
        @param port: port for connections
        """
        self.server = None
        self.client = None
        self.host = host
        self.port = port
        self.manager = Manager(consumer=self)

    async def reject_connection(self, websocket: WebSocketServerProtocol):
        """
        Rejects a connection
        @param websocket: websocket
        """
        await websocket.close(1008, "This RADI server can't accept more than one connection")

    async def handler(self, websocket: WebSocketServerProtocol):
        """
        Handles connection
        @param websocket: websocket
        """
        if self.client is not None and websocket != self.client:
            print("Client already connected, rejecting connection")
            await self.reject_connection(websocket)
        else:
            # self.client gets reassigned every time, but code is more clear
            # TODO: Just review this block of code
            self.client = websocket

        if self.client and self.client.closed:
            print("Client disconnected, machine state reset")
            self.manager.reset()
            self.client = None
            return

        async for websocket_message in websocket:
            try:
                s = json.loads(websocket_message)
                message = ManagerConsumerMessage(**s)
                await self.manager.trigger(message.command, data=message.data or None)
                response = {"message": f"Exercise state changed to {self.manager.state}"}
                await websocket.send(str(message.response(response)))
            except ManagerConsumerMessageException as e:
                await websocket.send(str(e))
            except Exception as e:
                if message is None:
                    ex = ManagerConsumerMessageException(message, str(e))
                else:
                    ex = ManagerConsumerMessageException(id=str(uuid4()), message=str(e))
                await websocket.send(str(ex))

    async def send_message(self, message_data):
        if self.client is not None and self.server is not None:
            message = ManagerConsumerMessage(id=str(uuid4()), command="state-changed", data=message_data)
            await self.client.send(str(message))

    def start(self):
        """
        Starts the consumer and listens for connections
        """
        self.server = websockets.serve(self.handler, self.host, self.port)
        print(f"Websocket server listening in {self.host}:{self.port}")
        asyncio.get_event_loop().run_until_complete(self.server)
        asyncio.get_event_loop().run_forever()


if __name__ == '__main__':
    consumer = ManagerConsumer('0.0.0.0', 7163)
    consumer.start()
