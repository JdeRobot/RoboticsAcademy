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

from parallelIce.threadSensor import ThreadSensor
from parallelIce.pose3dClient import Pose3D
from sensors.grid import Grid
import threading


class Sensor(threading.Thread):

    def __init__(self, grid, pose3d, start):
        self.grid = grid
        self.pose3d = pose3d
        if self.pose3d.hasproxy():
            self.grid.initPose(self.pose3d.getX(), self.pose3d.getY(), self.pose3d.getYaw())
        else:
            self.grid.initPose(0, 0, 0)

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
        if self.pose3d.hasproxy():
            self.pose3d.update()
            self.grid.updatePose(self.pose3d.getX(), self.pose3d.getY(), self.pose3d.getYaw())

        
    def setGetPathSignal(self, signal):
        self.getPathSig = signal


    def getPose3D(self):
        if self.pose3d.hasproxy():
            return self.pose3d.getPose3D()

    def getRobotX(self):
        if self.pose3d.hasproxy():
            return self.pose3d.getX()

    def getRobotY(self):
        if self.pose3d.hasproxy():
            return self.pose3d.getY()

    def getRobotTheta(self):
        if self.pose3d.hasproxy():
            return self.pose3d.Yaw()    