import threading
import time
from datetime import datetime

time_cycle = 80


class StoppableThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self, target, kill_event=threading.Event(), *args, **kwargs):
        super(StoppableThread, self).__init__(*args, **kwargs)
        self._target = target
        self._target_args = kwargs["args"]
        self._kill_event = kill_event

    def run(self):
        while not self.stopped():
            start_time = datetime.now()

            self._target(*self._target_args)

            finish_time = datetime.now()

            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            # print (ms)
            if ms < time_cycle:
                time.sleep((time_cycle - ms) / 1000.0)

    def stop(self):
        self._kill_event.set()

    def stopped(self):
        return self._kill_event.is_set()
