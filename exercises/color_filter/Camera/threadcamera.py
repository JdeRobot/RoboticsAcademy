#
# Created on Mar 7, 2017
#
# @author: dpascualhe
#
# Based on @nuriaoyaga code:
# https://github.com/RoboticsURJC-students/2016-tfg-nuria-oyaga/blob/
#     master/camera/threadcamera.py
#

import time
import threading
from datetime import datetime



class ThreadCamera(threading.Thread):



    def __init__(self, cam):
        ''' Threading class for Camera. '''

        self.t_cycle = 50      # ms

        self.cam = cam
        threading.Thread.__init__(self)

    def run(self):
        ''' Updates the thread. '''
        while(True):
            start_time = datetime.now()
            self.cam.update()
            end_time = datetime.now()

            dt = end_time - start_time
            dtms = ((dt.days * 24 * 60 * 60 + dt.seconds) * 1000
                + dt.microseconds / 1000.0)

            delta = max(self.t_cycle, dtms)
            self.framerate = int(1000.0 / delta)

            if(dtms < self.t_cycle):
                time.sleep((self.t_cycle - dtms) / 1000.0)
