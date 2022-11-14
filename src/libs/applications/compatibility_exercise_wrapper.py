import os.path
import subprocess
import time

from src.libs.process_utils import stop_process_and_children
from src.logging.log_manager import LogManager
from src.manager.application.robotics_python_application_interface import IRoboticsPythonApplication
from websocket import create_connection

logger = LogManager.logger


class CompatibilityExerciseWrapper(IRoboticsPythonApplication):
    def __init__(self, exercise_command, gui_command):
        home_dir = os.path.expanduser('~')

        # TODO: review hardcoded values
        process_ready, self.exercise_server = self._run_exercise_server(f"python {exercise_command}",
            f'{home_dir}/ws_code.log',
            'websocket_code=ready')
        if process_ready:
            logger.info(f"Exercise code {exercise_command} launched")
            time.sleep(1)
            self.exercise_connection = create_connection('ws://127.0.0.1:1905')
        else:
            self.exercise_server.kill()
            raise RuntimeError(f"Exercise {exercise_command} could not be run")

        process_ready, self.gui_server = self._run_exercise_server(f"python {gui_command}", f'{home_dir}/ws_gui.log',
                                                                   'websocket_gui=ready')
        if process_ready:
            logger.info(f"Exercise gui {gui_command} launched")
            time.sleep(1)
            self.gui_connection = create_connection('ws://127.0.0.1:2303')
        else:
            self.gui_server.kill()
            raise RuntimeError(f"Exercise GUI {gui_command} could not be run")

    def _run_exercise_server(self, cmd, log_file, load_string, timeout: int = 5):
        process = subprocess.Popen(f"exec {cmd}", shell=True, stdout=subprocess.PIPE, bufsize=1024, universal_newlines=True)

        process_ready = False
        while not process_ready:
            try:
                f = open(log_file, "r")
                if f.readline() == load_string:
                    process_ready = True
                f.close()
                time.sleep(0.2)
            except Exception as e:
                print(f"waiting for server string '{load_string}'...")
                time.sleep(0.2)

        return process_ready, process

    def run(self):
        pass

    def stop(self):
        pass

    def restart(self):
        pass

    @property
    def is_alive(self):
        pass

    def load_code(self, code: str):
        self.connection.send()

    def terminate(self):
        stop_process_and_children(self.exercise_server)
        stop_process_and_children(self.gui_server)
