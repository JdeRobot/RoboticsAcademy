import logging
import os

from src.libs.process_utils import classproperty, singleton


@singleton
class LogManager:
    def __init__(self):
        log_path = os.getcwd()
        log_level = logging.INFO
        log_to_console = True

        self.log_file = os.path.join(log_path, "ram.log")
        self.logFormatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s] (%(name)s)  %("
                                              "message)s")
        self.logger = logging.getLogger()
        self.logger.setLevel(log_level)

        self.fileHandler = logging.FileHandler(self.log_file)
        self.fileHandler.setFormatter(self.logFormatter)
        self.logger.addHandler(self.fileHandler)

        if log_to_console:
            self.consoleHandler = logging.StreamHandler()
            self.consoleHandler.setFormatter(self.logFormatter)
            self.logger.addHandler(self.consoleHandler)


