# Functions to start and close console
import os
import sys


def start_console():
    # Get all the file descriptors and choose the latest one
    fds = os.listdir("/dev/pts/")
    console_fd = str(max(map(int, fds[:-1])))

    sys.stderr = open('/dev/pts/' + console_fd, 'w')
    sys.stdout = open('/dev/pts/' + console_fd, 'w')
    sys.stdin = open('/dev/pts/' + console_fd, 'w')


def close_console():
    sys.stderr.close()
    sys.stdout.close()
    sys.stdin.close()
