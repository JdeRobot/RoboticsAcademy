'''
Do not run manager_utils.py as a stand-alone script
Contains utility functions for the manager script
'''

import os, subprocess
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

    def run(self):
        subprocess.Popen(self.cmd, shell=True, stdout=subprocess.PIPE, bufsize=1024, universal_newlines=True)

    def call(self):
        subprocess.call(self.cmd, shell=True, stdout=subprocess.PIPE, bufsize=1024, universal_newlines=True) 

# For debugging log output
class DockerThreadLog(threading.Thread):
    def __init__(self, cmd):
        threading.Thread.__init__(self)
        self.cmd = cmd

    def run(self):
        log = open("/RoboticsAcademy/logfile", "w", 1)
        subprocess.Popen(self.cmd, shell=True, stdout=log, stderr=subprocess.STDOUT, bufsize=1024, universal_newlines=True)

