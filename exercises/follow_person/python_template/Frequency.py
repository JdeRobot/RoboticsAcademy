import time
from datetime import datetime

start_time_internal_freq_control = None


def tick(ideal_cycle: int = 50):
    global start_time_internal_freq_control
    finish_time = datetime.now()
    ideal_ms = 1000 / ideal_cycle

    if start_time_internal_freq_control is None:
        start_time_internal_freq_control = finish_time
        return

    dt = finish_time - start_time_internal_freq_control
    ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0

    if ms < ideal_ms:
        time.sleep((ideal_ms - ms) / 1000.0)

    start_time_internal_freq_control = finish_time
