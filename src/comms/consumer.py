import asyncio
import json

import websockets
from websockets.server import WebSocketServerProtocol

from src.comms.consumer_message import ManagerConsumerMessage, ManagerConsumerMessageException
from src.exercise_manager import ExerciseManager


class ManagerConsumer:
    """
    Robotics Academy websocket consumer
    """

    def __init__(self, host, port):
        """
        Initializes a new ManagerConsumer
        @param host: host for connections, '0.0.0.0' to bind all interfaces
        @param port: port for connections
        """
        self.server = None
        self.client = None
        self.host = host
        self.port = port
        self.manager = ExerciseManager()

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
        if self.client is None or self.client.closed:
            if self.client and self.client.closed:
                self.manager.reset()
            self.client = websocket
        else:
            if websocket != self.client:
                await self.reject_connection(websocket)
                return

        async for websocket_message in websocket:
            try:
                message = ManagerConsumerMessage.from_str(websocket_message)
                self.manager.trigger(message.command)
                response = {"message": f"Exercise state changed to {self.manager.state}"}
                await websocket.send(str(message.response(response)))
            except ManagerConsumerMessageException as e:
                await websocket.send(str(e))
            except Exception as e:
                ex = ManagerConsumerMessageException(message, str(e))
                await websocket.send(str(ex))

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
