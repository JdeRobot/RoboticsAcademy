import json
import signal
import subprocess
import sys
import threading
import time
import importlib
from threading import Thread

from src.manager.libs.applications.compatibility.client import Client
from src.manager.libs.process_utils import stop_process_and_children
from src.manager.ram_logging.log_manager import LogManager
from src.manager.manager.application.robotics_python_application_interface import IRoboticsPythonApplication
from src.manager.manager.lint.linter import Lint


class CompatibilityExerciseWrapper():
    def __init__(self):
        self.running = False
        self.linter = Lint()
        self.brain_ready_event = threading.Event()
        self.pick = None
        self.exercise = None
        self.run() 

    def save_pick(self, pick):
        self.pick = pick

    def send_pick(self, pick):
        self.gui_connection.send("#pick" + json.dumps(pick))
        print("#pick" + json.dumps(pick))

    def handle_client_gui(self, msg):
        if msg['msg'] == "#pick":
            self.pick = msg['data']
        else:
            self.gui_connection.send(msg['msg'])

    def _run_server(self, cmd):
        process = subprocess.Popen(f"{cmd}", shell=True, stdout=sys.stdout, stderr=subprocess.STDOUT,
                                   bufsize=1024, universal_newlines=True)
        return process

    def run(self):
        self.exercise = self._run_server(f"python3 $EXERCISE_FOLDER/entry_point/exercise.py")

    def stop(self):
        pass

    def resume(self):
        pass

    def pause(self):
        pass

    @property
    def is_alive(self):
        return self.running

    def terminate(self):
                
        if self.exercise is not None:
            stop_process_and_children(self.exercise)

        self.running = False
