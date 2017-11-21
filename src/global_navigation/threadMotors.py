#
#  Copyright (C) 1997-2015 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#        Samuel Rey Escudero <samuel.rey.escudero@gmail.com>
#
import threading, time
from datetime import datetime

time_cycle = 80;


class Velocity():
    def __init__(self, v, w, maxV, maxW):
        self.v = v
        self.w = w
        self.maxV = maxV
        self.maxW = maxW
        self.lock = threading.Lock()

    def setV(self, v):
        self.lock.acquire()
        self.v = v
        self.lock.release()

    def setW(self, w):
        self.lock.acquire()
        self.w = w
        self.lock.release()

    def getV(self):
        self.lock.acquire()
        tmp = self.v
        self.lock.release()
        return tmp

    def getW(self):
        self.lock.acquire()
        tmp = self.w
        self.lock.release()
        return tmp

    def getMaxV(self):
        return self.maxV

    def getMaxW(self):
        return self.maxW


class ThreadMotors(threading.Thread):

    def __init__(self, motors, vel):
        self.motors = motors
        self.velocity = vel
        threading.Thread.__init__(self)

    def run(self):
        while (True):

            start_time = datetime.now()
            self.motors.sendV(self.velocity.getV())
            self.motors.sendW(self.velocity.getW())
            #self.motors.sendVelocities()

            finish_Time = datetime.now()

            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0);
