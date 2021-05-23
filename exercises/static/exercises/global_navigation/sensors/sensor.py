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
#       Samuel Rey Escudero <samuel.rey.escudero@gmail.com>
#       Alberto Martin Florido <almartinflorido@gmail.com>
#

from threadSensor import ThreadSensor
from sensors.grid import Grid
import threading


class Sensor(threading.Thread):

    def __init__(self, grid, pose3d, start):
        self.grid = grid
        self.pose3d = pose3d
        self.grid.initPose(self.pose3d.getPose3d().x, 
                           self.pose3d.getPose3d().y, 
                           self.pose3d.getPose3d().yaw)
       
        self.kill_event = threading.Event()
        self.thread = ThreadSensor(self, self.kill_event)
        self.thread.daemon = True

        if start:
            self.start()

    # if client is stopped you can not start again, Threading.Thread raised error
    def start(self):
        self.kill_event.clear()
        self.thread.start()

    # if client is stopped you can not start again
    def stop(self):
        self.kill_event.set()


    def update(self):
        self.grid.updatePose(self.pose3d.getPose3d().x, 
                             self.pose3d.getPose3d().y, 
                             self.pose3d.getPose3d().yaw)

        
    def setGetPathSignal(self, signal):
        self.getPathSig = signal


    def getPose3D(self):
        return self.pose3d.getPose3d()

    def getRobotX(self):
        return self.pose3d.getPose3d().x

    def getRobotY(self):
        return self.pose3d.getPose3d().y

    def getRobotTheta(self):
        return self.pose3d.getPose3d().yaw    
