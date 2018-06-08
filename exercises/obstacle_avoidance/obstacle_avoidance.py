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

import sys, os
from PyQt5.QtWidgets import QApplication
from gui.GUI import MainWindow
from gui.threadGUI import ThreadGUI
from MyAlgorithm import MyAlgorithm

from interfaces.camera import ListenerCamera
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser
from interfaces.motors import PublisherMotors


if __name__ == "__main__":

    camera = ListenerCamera("/F1ROS/cameraL/image_raw")
    motors = PublisherMotors("/F1ROS/cmd_vel", 3, 0.5)
    pose3d = ListenerPose3d("/F1ROS/odom")
    laser = ListenerLaser("/F1ROS/laser/scan")

    algorithm=MyAlgorithm(pose3d, laser, motors)

    app = QApplication(sys.argv)
    myGUI = MainWindow()
    myGUI.setCamera(camera)
    myGUI.setMotors(motors)
    myGUI.setPose3D(pose3d)
    myGUI.setLaser(laser)
    myGUI.setAlgorithm(algorithm)
    myGUI.show()


    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()

    id = app.exec_()
    os._exit(id)

