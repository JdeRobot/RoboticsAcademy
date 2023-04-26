import logging
import os

from src.libs.process_utils import classproperty, singleton


@singleton
class LogManager:
    def __init__(self):
        self.log_path = os.getcwd()
        self.log_level = logging.INFO
        self.log_to_console = True

        self.log_file = os.path.join(self.log_path, "ram.log")
        self.logFormatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s] (%(name)s)  %("
                                              "message)s")
        self.logger = logging.getLogger()
        self.logger.setLevel(self.log_level)

        self.fileHandler = logging.FileHandler(self.log_file)
        self.fileHandler.setFormatter(self.logFormatter)
        self.logger.addHandler(self.fileHandler)

        if self.log_to_console:
            self.consoleHandler = logging.StreamHandler()
            self.consoleHandler.setFormatter(self.logFormatter)
            self.logger.addHandler(self.consoleHandler)

    def getLogger(self, logger):
        return logging.getLogger(logger)