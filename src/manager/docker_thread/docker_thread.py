"""Docker Thread Class"""
import threading
import subprocess
import os
import signal




class DockerThread(threading.Thread):
    """Threaded Docker Thread Class"""
    def __init__(self, cmd, shell=True):
        threading.Thread.__init__(self)
        self.cmd = cmd
        self.process = None
        self.shell=shell

    def run(self):
        self.process = subprocess.Popen(self.cmd, shell=self.shell, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True,
                         bufsize=1024, universal_newlines=True)
        self.process.communicate()

    def terminate(self):
        """Terminates the thread and the process"""
        if self.process:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except ProcessLookupError as error:
                print(f"{self.process.pid}: Process already terminated {error}")
               