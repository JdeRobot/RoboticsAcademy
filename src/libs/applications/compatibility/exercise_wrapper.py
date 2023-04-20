import json
import logging
import os.path
import subprocess
import sys
import threading
import time
import rosservice
from threading import Thread

from src.libs.applications.compatibility.client import Client
from src.libs.process_utils import stop_process_and_children
from src.ram_logging.log_manager import LogManager
from src.manager.application.robotics_python_application_interface import IRoboticsPythonApplication
from src.manager.lint.linter import Lint


class CompatibilityExerciseWrapper(IRoboticsPythonApplication):
    def __init__(self, exercise_command, gui_command, update_callback):
        super().__init__(update_callback)

        home_dir = os.path.expanduser('~')
        self.running = False
        self.linter = Lint()
        self.brain_ready_event = threading.Event()
        # TODO: review hardcoded values
        process_ready, self.exercise_server = self._run_exercise_server(f"python {exercise_command}",
                                                                        f'{home_dir}/ws_code.log',
                                                                        'websocket_code=ready')
        if process_ready:
            LogManager.logger.info(
                f"Exercise code {exercise_command} launched")
            time.sleep(1)
            self.exercise_connection = Client(
                'ws://127.0.0.1:1905', 'exercise', self.server_message)
            self.exercise_connection.start()
        else:
            self.exercise_server.kill()
            raise RuntimeError(f"Exercise {exercise_command} could not be run")

        process_ready, self.gui_server = self._run_exercise_server(f"python {gui_command}", f'{home_dir}/ws_gui.log',
                                                                   'websocket_gui=ready')
        if process_ready:
            LogManager.logger.info(f"Exercise gui {gui_command} launched")
            time.sleep(1)
            self.gui_connection = Client(
                'ws://127.0.0.1:2303', 'gui', self.server_message)
            self.gui_connection.start()
        else:
            self.gui_server.kill()
            raise RuntimeError(f"Exercise GUI {gui_command} could not be run")
        
        self.running = True

        self.start_send_freq_thread()


    def send_freq(self, exercise_connection, is_alive):
        """Send the frequency of the brain and gui to the exercise server"""
        while is_alive():
            exercise_connection.send(
                """#freq{"brain": 20, "gui": 10, "rtf": 100}""")
            time.sleep(1)

    def start_send_freq_thread(self):
        """Start a thread to send the frequency of the brain and gui to the exercise server"""
        daemon = Thread(target=lambda: self.send_freq(self.exercise_connection,
                        lambda: self.is_alive), daemon=False, name='Monitor frequencies')
        daemon.start()

    def _run_exercise_server(self, cmd, log_file, load_string, timeout: int = 5):
        process = subprocess.Popen(f"{cmd}", shell=True, stdout=sys.stdout, stderr=subprocess.STDOUT,
                                   bufsize=1024, universal_newlines=True)

        process_ready = False
        while not process_ready:
            try:
                f = open(log_file, "r")
                if f.readline() == load_string:
                    process_ready = True
                f.close()
                time.sleep(0.2)
            except Exception as e:
                LogManager.logger.debug(
                    f"waiting for server string '{load_string}'...")
                time.sleep(0.2)

        return process_ready, process

    def server_message(self, name, message):
        if name == "gui":  # message received from GUI server
            LogManager.logger.debug(
                f"Message received from gui: {message[:30]}")
            self._process_gui_message(message)
        elif name == "exercise":  # message received from EXERCISE server
            if message.startswith("#exec"):
                self.brain_ready_event.set()
            LogManager.logger.info(
                f"Message received from exercise: {message[:30]}")
            self._process_exercise_message(message)

    def _process_gui_message(self, message):
        payload = json.loads(message[4:])
        self.update_callback(payload)
        self.gui_connection.send("#ack")

    def _process_exercise_message(self, message):
        payload = json.loads(message[5:])
        self.update_callback(payload)
        self.exercise_connection.send("#ack")

    def run(self):
        rosservice.call_service("/gazebo/unpause_physics", [])

    def stop(self):
        rosservice.call_service('/gazebo/pause_physics', [])
        rosservice.call_service("/gazebo/reset_world", [])

    def resume(self):
        rosservice.call_service("/gazebo/unpause_physics", [])

    def pause(self):
        rosservice.call_service('/gazebo/pause_physics', [])

    def restart(self):
        pass

    @property
    def is_alive(self):
        return self.running

    def load_code(self, code: str):
        errors = self.linter.evaluate_code(code)
        if errors == "":
            self.brain_ready_event.clear()
            self.exercise_connection.send(f"#code {code}")
            self.brain_ready_event.wait()
        else:
            raise Exception(errors)

    def terminate(self):
        self.running = False
        self.exercise_connection.stop()
        self.gui_connection.stop()

        stop_process_and_children(self.exercise_server)
        stop_process_and_children(self.gui_server)
