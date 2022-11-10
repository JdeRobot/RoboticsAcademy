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
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#       Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>
#  Rosified by:
#       Francisco Perez Salgado <f.perez475@gmail.com>
#


# General imports
import sys

# Practice imports
from gui.GUI import MainWindow
from gui.threadGUI import ThreadGUI
from MyAlgorithm import MyAlgorithm
from PyQt5.QtWidgets import QApplication
from interfaces.camera import ListenerCamera
from interfaces.motors import PublisherMotors
from interfaces.point import ListenerPoint, PublisherPoint

if __name__ == "__main__":

    cameraL = ListenerCamera("/TurtlebotROS/cameraL/image_raw")
    cameraR = ListenerCamera("/TurtlebotROS/cameraR/image_raw")
    #motors = PublisherMotors("/TurtlebotROS/cmd_vel", 4, 0.3)
    
    pointTopic = "/TurtlebotROS/point"
    publishPoint = PublisherPoint(pointTopic)
    listenPoint = ListenerPoint(pointTopic)
    
    algorithm=MyAlgorithm(cameraL, cameraR, publishPoint)

    app = QApplication(sys.argv)
    myGUI = MainWindow()
    myGUI.setCamera(cameraL, 'left')
    myGUI.setCamera(cameraR, 'right')
    myGUI.setPlot(listenPoint)
    myGUI.setAlgorithm(algorithm)
    myGUI.show()


    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()


    sys.exit(app.exec_())
