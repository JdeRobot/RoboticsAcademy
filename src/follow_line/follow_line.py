#!/usr/bin/python
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
#       Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>
#


import sys
from PyQt4 import QtCore, QtGui
from gui.GUI import MainWindow
from gui.threadGUI import ThreadGUI
from sensors.camera import Camera
from actuators.motors import Motors
from sensors.threadSensor import ThreadSensor
import easyiceconfig as EasyIce
from MyAlgorithm import MyAlgorithm




if __name__ == "__main__":
    #sensor = Sensor()
    ic = EasyIce.initialize(sys.argv)
    cameraL = Camera(ic, "FollowLine.CameraLeft")
    cameraR = Camera(ic, "FollowLine.CameraRight")
    motors = Motors (ic, "FollowLine.Motors")
    algorithm=MyAlgorithm(cameraL)

    app = QtGui.QApplication(sys.argv)
    myGUI = MainWindow()
    myGUI.setCameraL(cameraL)
    myGUI.setCameraR(cameraR)
    myGUI.setMotors(motors)
    myGUI.setAlgorithm(algorithm)
    myGUI.show()

    t1 = ThreadSensor(cameraL)
    t1.daemon=True
    t1.start()

    t3 = ThreadSensor(cameraR)
    t3.daemon=True
    t3.start()


    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()


    sys.exit(app.exec_())
