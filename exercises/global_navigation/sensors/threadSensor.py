import threading, time
from datetime import datetime

time_cycle = 80

class ThreadSensor(threading.Thread):

    def __init__(self, sensor, kill_event):
        self.sensor = sensor
        self.kill_event = kill_event
        threading.Thread.__init__(self, args=kill_event)

    def run(self):
        while (not self.kill_event.is_set()):
            start_time = datetime.now()

            self.sensor.update()

            finish_Time = datetime.now()

            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            #print (ms)
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)