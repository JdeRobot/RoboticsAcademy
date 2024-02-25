import logging
import os

from src.manager.libs.singleton import singleton


# Clase para un Formatter personalizado que añade colores
class ColorFormatter(logging.Formatter):
    # Diccionario de colores para diferentes niveles de log
    COLORS = {
        logging.ERROR: "\033[91m",  # Rojo para errores
        logging.WARNING: "\033[93m",  # Amarillo para warnings
        logging.INFO: "\033[0m",  # Verde para info
        logging.DEBUG: "\033[96m",  # Cyan para debug
    }
    RESET = "\033[0m"  # Resetear a color por defecto

    def format(self, record):
        color = self.COLORS.get(record.levelno)
        message = super().format(record)
        if color:
            message = f"{color}{message}{self.RESET}"
        return message


@singleton
class LogManager:
    def __init__(self):
        log_path = os.getcwd()
        log_level = logging.INFO
        log_to_console = True

        self.log_file = os.path.join(log_path, "ram.log")
        log_format = "%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s] (%(name)s)  %(message)s"
        date_format = "%H:%M:%S"
        self.log_formatter = logging.Formatter(log_format, date_format)
        self.color_formatter = ColorFormatter(
            log_format, date_format)  # Formatter con color

        self.logger = logging.getLogger('my_app_logger')
        self.logger.setLevel(log_level)
        self.logger.propagate = False

        self.file_handler = logging.FileHandler(self.log_file)
        self.file_handler.setFormatter(self.log_formatter)
        self.logger.addHandler(self.file_handler)

        if log_to_console:
            self.console_handler = logging.StreamHandler()
            # Usar formatter con color para la consola
            self.console_handler.setFormatter(self.color_formatter)
            self.logger.addHandler(self.console_handler)
