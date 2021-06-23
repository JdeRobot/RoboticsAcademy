'''
Do not run manager_utils.py as a stand-alone script
Contains utility functions for the manager script
'''

import os
import stat
import threading


# Function to check if a device exists
def check_device(device_path):
    try:
        return stat.S_ISCHR(os.lstat(device_path)[stat.ST_MODE])
    except:
        return False


def quieten(cmd):
    '''
    We use "> /dev/null 2>&1 &" to quieten vnc
    2(stderr) stream is combined with &1(stdout)
    and redirected to /dev/null
    '''
    cmd += " > /dev/null 2>&1 &"
    return cmd


# Docker Thread class for running commands on threads
class DockerThread(threading.Thread):
    def __init__(self, cmd):
        threading.Thread.__init__(self)
        self.cmd = cmd
        self.out = None

    def run(self):
        stream = os.popen(self.cmd)
        output = stream.read()
        self.out = output

    def print_output(self):
        return self.out


