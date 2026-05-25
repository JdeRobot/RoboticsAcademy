from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console

# Graphical User Interface Class


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)
        self.start()


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

# Redirect the console
start_console()
