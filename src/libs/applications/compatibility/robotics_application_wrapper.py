import os.path
import subprocess
import sys
import time

import psutil

from src.libs.process_utils import stop_process_and_children
from src.manager.application.robotics_python_application_interface import IRoboticsPythonApplication
from src.manager.lint.linter import Lint
from src.manager.docker_thread.docker_thread import DockerThread

from src.libs.applications.compatibility.client import Client
from src.libs.process_utils import stop_process_and_children
from src.ram_logging.log_manager import LogManager
from src.manager.application.robotics_python_application_interface import IRoboticsPythonApplication
from src.manager.lint.linter import Lint

import os

class RoboticsApplicationWrapper(IRoboticsPythonApplication):

    def __init__(self, update_callback):
        super().__init__(update_callback)
        self.running = False
        self.linter = Lint()
        time.sleep(5)
        self.start_console()
        self.user_process = None

    def _create_process(self, cmd):
        #print("creando procesos")
        process = subprocess.Popen(f"{cmd}", shell=True, stdout = sys.stdout, stderr=subprocess.STDOUT, bufsize=1024, universal_newlines=True)
        psProcess = psutil.Process(pid=process.pid)
        return psProcess

    def terminate(self):
        self.close_console()
        self.running = False
        stop_process_and_children(self.user_process)
        self.user_process = None

    def load_code(self, code: str):
        self.f = open("user_code.py", "w")
        self.f.write(code)
        self.f.close()
        #self.update_callback("Hello World")

    def run(self):     
        self.user_process = self._create_process(f"DISPLAY=:2 python {self.f.name}")

        self.running = True

    def stop(self):
        stop_process_and_children(self.user_process)
        self.user_process = None

    def restart(self):
        pass

    def resume(self):
        self.suspend_resume("resume")

    def pause(self):
        if self.user_process != None:
            self.suspend_resume("pause")

    @property
    def is_alive(self):
        return self.running

    def start_console(self):
        # Get all the file descriptors and choose the latest one
        fds = os.listdir("/dev/pts/")
        fds.sort()
        console_fd = fds[-2]

        sys.stderr = open('/dev/pts/' + console_fd, 'w')
        sys.stdout = open('/dev/pts/' + console_fd, 'w')
        sys.stdin = open('/dev/pts/' + console_fd, 'w')

    def close_console(self):
        sys.stderr.close()
        sys.stdout.close()
        sys.stdin.close()

    def suspend_resume(self, signal):
        # collect processes to stop
        children = self.user_process.children(recursive=True)
        children.append(self.user_process)

        # send signal to processes
        for p in children:
            try:
                if(signal == "pause"):
                    p.suspend()
                if(signal == "resume"):
                    p.resume()
            except psutil.NoSuchProcess:
                pass