import time
from datetime import datetime


class Frequency:
    start_time_internal_freq_control = 0

    def __init__(self):
        pass

    def tick(self):
        self.start_time = datetime.now()

    def tack(self, ideal_cycle: int = 20):
        self.finish_time = datetime.now()
        dt = self.finish_time - self.start_time
        ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0

        if ms < ideal_cycle:
            time.sleep((ideal_cycle - ms) / 1000.0)


frequency = Frequency()


def tick():
    frequency.tick()


def tack(ideal_cycle: int = 20):
    frequency.tack(ideal_cycle)
