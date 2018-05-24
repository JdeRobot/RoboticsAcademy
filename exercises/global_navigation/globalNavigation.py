#!/usr/bin/python3
# -*- coding: utf-8 -*-

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

import sys
import config
import comm
from PyQt5.QtWidgets import QApplication

from MyAlgorithm import MyAlgorithm
from gui.threadGUI import ThreadGUI
from gui.GUI import MainWindow
from threadMotors import ThreadMotors
from threadMotors import Velocity

import easyiceconfig as EasyIce
from parallelIce.pose3dClient import Pose3D
from parallelIce.motors import Motors
from sensors.sensor import Sensor
from sensors.grid import Grid

import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)


def removeMapFromArgs():
    for arg in sys.argv:
        if (arg.split(".")[1] == "conf"):
            sys.argv.remove(arg)


if __name__ == '__main__':

    if len(sys.argv) < 2:
        print('ERROR: python2 globalNavigation.py [MAP CONFIG file] [YAML CONFIG file]')
        sys.exit(-1)

    app = QApplication(sys.argv)
    frame = MainWindow()
    grid = Grid(frame)

    removeMapFromArgs()

    cfg = config.load(sys.argv[1])
    #starting comm
    jdrc= comm.init(cfg, 'TeleTaxi')

    motors = jdrc.getMotorsClient ("TeleTaxi.Motors")
    pose = jdrc.getPose3dClient("TeleTaxi.Pose3D")

    vel = Velocity(0, 0, motors.getMaxV(), motors.getMaxW())

    frame.setVelocity(vel)
    sensor = Sensor(grid, pose, True)
    sensor.setGetPathSignal(frame.getPathSig)
    frame.setGrid(grid)
    frame.setSensor(sensor)
    algorithm = MyAlgorithm(grid, sensor, vel)
    frame.setAlgorithm(algorithm)
    frame.show()
    
    t1 = ThreadMotors(motors, vel)
    t1.daemon = True
    t1.start()

    t2 = ThreadGUI(frame)  
    t2.daemon = True
    t2.start()
    
    sys.exit(app.exec_()) 
