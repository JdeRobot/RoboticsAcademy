import threading
import websocket

from src.ram_logging.log_manager import LogManager


class Client(threading.Thread):
    def __init__(self, url, name, callback):
        super().__init__()
        self.name = name
        self.callback = callback
        self._stop = threading.Event()
        self.client = websocket.WebSocketApp(
            url,
            on_message=self.on_message,
            on_close=self.on_close,
            on_error=self.on_error,
            on_open=self.on_open)

    def run(self) -> None:
        try:
            while True:
                self.client.run_forever(ping_timeout=None, ping_interval=0)
                if self._stop.isSet():
                    return
        except Exception as ex:
            LogManager.logger.exception(ex)

    def stop(self) -> None:
        self._stop.set()
        self.client.close()

    def send(self, data):
        self.client.send(data)

    def on_message(self, ws, message):
        self.callback(self.name, message)

    def on_error(self, ws, error):
        LogManager.logger.error(error)

    def on_close(self, ws, status, msg):
        LogManager.logger.info(f"Connection with {self.name} closed, status code: {status}, close message: {msg}")

    def on_open(self, ws):
        LogManager.logger.info(f"Connection with {self.name} opened")
