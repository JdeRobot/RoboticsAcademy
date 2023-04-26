import threading
import subprocess


class DockerThread(threading.Thread):
    def __init__(self, cmd, shell: bool = False):
        threading.Thread.__init__(self)
        self.cmd = cmd
        self.shell = shell
        self.process = None

    def run(self):
        self.process = subprocess.Popen(self.cmd, shell=self.shell, stdout=subprocess.PIPE,
                                        bufsize=1024, universal_newlines=True)
        self.process.wait()

    def terminate(self):
        if self.process:
            self.process.kill()

