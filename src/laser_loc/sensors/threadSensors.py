import time
import threading
from datetime import datetime

time_cycle = 20  # ms

class ThreadSensors(threading.Thread):

    def __init__(self, sensors):
        ''' Threading class for Sensors. '''
        self.sensors = sensors
        threading.Thread.__init__(self)

    def run(self):
        ''' Updates the thread. '''
        while(True):
            start_time = datetime.now()
            self.sensors.update()
            end_time = datetime.now()

            dt = end_time - start_time
            dtms = ((dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0)

            if(dtms < time_cycle):
                time.sleep((time_cycle - dtms) / 1000.0);
