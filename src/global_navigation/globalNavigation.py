#!/usr/bin/python
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
#       Alberto Martin Florido <almartinflorido@gmail.com>
#

import sys
from MyAlgorithm import MyAlgorithm
import easyiceconfig as EasyIce
from parallelIce.pose3dClient import Pose3D
from parallelIce.motors import Motors
from sensors.sensor import Sensor
from sensors.grid import Grid
from gui.threadGUI import ThreadGUI
from gui.GUI import MainWindow
from PyQt4 import QtGui

import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print >> sys.stderr, 'ERROR: python main.py --mapConfig=[map config file] --Ice.Config=[ice file]'
        #sys.exit(-1)


    ic = EasyIce.initialize(sys.argv)
    pose = Pose3D (ic, "Introrob.Pose3D")
    motors = Motors (ic, "FollowLine.Motors")



    app = QtGui.QApplication(sys.argv)
    frame = MainWindow()
    frame.setMotors(motors)
    frame.show()

    grid = Grid(frame)
    frame.setGrid(grid)

    sensor = Sensor(grid, pose)
    frame.setSensor(sensor)

    algorithm=MyAlgorithm(grid, sensor, motors)
    frame.setAlgorithm(algorithm)
    
    
    t2 = ThreadGUI(frame)  
    t2.daemon=True
    t2.start()
    
    sys.exit(app.exec_()) 

