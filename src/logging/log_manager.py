import logging
import os

from src.libs.process_utils import classproperty


class LogManager:
    _instance = None

    def __new__(cls, log_path: str = None, log_to_console: bool = True, loglevel=logging.WARNING):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            if log_path is None:
                log_path = os.getcwd()
            else:
                os.makedirs(log_path, exist_ok=True)

            cls._instance.log_file = os.path.join(log_path, "ram.log")
            cls._instance.logFormatter = logging.Formatter(
                "%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s")
            cls._instance.rootLogger = logging.getLogger()
            cls._instance.rootLogger.setLevel(loglevel)

            cls._instance.fileHandler = logging.FileHandler(cls._instance.log_file)
            cls._instance.fileHandler.setFormatter(cls._instance.logFormatter)
            cls._instance.rootLogger.addHandler(cls._instance.fileHandler)

            if log_to_console:
                cls._instance.consoleHandler = logging.StreamHandler()
                cls._instance.consoleHandler.setFormatter(cls._instance.logFormatter)
                cls._instance.rootLogger.addHandler(cls._instance.consoleHandler)
        return cls._instance

    @classproperty
    def logger(cls) -> logging.Logger:
        if cls._instance is None:
            cls._instance = LogManager()
        return cls._instance.rootLogger


