#!/usr/bin/python3
#
#  Copyright (C) 1997-2016 JDE Developers Team
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
#       Eduardo Perdices <eperdices@gsyc.es>
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#

import sys, os, config
import rospy
import comm

from gui.GUI import MainWindow
from gui.threadGUI import ThreadGUI
from MyAlgorithm import MyAlgorithm
from PyQt5.QtWidgets import QApplication

if __name__ == "__main__":

    cfg = config.load(sys.argv[1])

    jdrc= comm.init(cfg, 'Autopark')
    motors = jdrc.getMotorsClient("Autopark.Motors")
    pose3d = jdrc.getPose3dClient("Autopark.Pose3D")
    laser1 = jdrc.getLaserClient("Autopark.Laser1")
    laser2 = jdrc.getLaserClient("Autopark.Laser2")
    laser3 = jdrc.getLaserClient("Autopark.Laser3")

    algorithm=MyAlgorithm(pose3d, laser1, laser2, laser3, motors)

    app = QApplication(sys.argv)
    myGUI = MainWindow()
    myGUI.setMotors(motors)
    myGUI.setPose3D(pose3d)
    myGUI.setLaser1(laser1)
    myGUI.setLaser2(laser2)
    myGUI.setLaser3(laser3)
    myGUI.setAlgorithm(algorithm)
    myGUI.show()


    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()


    id = app.exec_()
    os._exit(id)
