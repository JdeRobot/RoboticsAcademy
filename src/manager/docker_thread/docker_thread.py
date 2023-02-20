import threading
import subprocess


class DockerThread(threading.Thread):
    def __init__(self, cmd):
        threading.Thread.__init__(self)
        self.cmd = cmd

    def run(self):
        subprocess.Popen(self.cmd, shell=True, stdout=subprocess.PIPE,
                         bufsize=1024, universal_newlines=True)

    def call(self):
        subprocess.call(self.cmd, shell=True, stdout=subprocess.PIPE,
                        bufsize=1024, universal_newlines=True)
