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
#       Alberto Martin Florido <almartinflorido@gmail.com>
#
import threading, time
from datetime import datetime

time_cycle = 80


class ThreadSensor(threading.Thread):

    def __init__(self, sensor, kill_event):
        self.sensor = sensor
	self.kill_event = kill_event
	threading.Thread.__init__(self, args=kill_event)

    def run(self):
        while (True):

            start_time = datetime.now()

            self.sensor.update()

            #if (self.sensor.isPlayButton()):
            #    self.algorithm.execute();

            finish_Time = datetime.now()

            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)
