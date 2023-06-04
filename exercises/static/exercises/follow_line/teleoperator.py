import threading
import queue
import json
from datetime import datetime
import time

class TeleopThread(threading.Thread):
    def __init__(self, q, exit_signal, hal):
        super(TeleopThread, self).__init__()
        # Init thread queue
        self.q = q
        # Init exit_signal
        self.exit_signal = exit_signal
        # Init v and w
        self.v = 0
        self.w = 0
        # Init hal
        self.hal = hal


    def run(self):
        while(True):
            if not self.exit_signal.is_set():
                try:
                    # Get params from the queue
                    params = self.q.get(timeout=0)

                    # Check if parameters change
                    if (self.v != params["v"]) : self.v = params["v"]
                    if (self.w != params["w"]) : self.w = params["w"]
                except queue.Empty:
                    pass

                # Send instructions to HAL
                self.hal.motors.sendV(self.v)
                self.hal.motors.sendW(self.w)
