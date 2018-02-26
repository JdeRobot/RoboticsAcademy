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

import sys
import comm
import config
from PyQt5.QtWidgets import QApplication
from gui.GUI import MainWindow
from gui.threadGUI import ThreadGUI
#from parallelIce.motors import Motors
#from parallelIce.pose3dClient import Pose3DClient
#from parallelIce.cameraClient import CameraClient
#from parallelIce.laserClient import LaserClient
#import easyiceconfig as EasyIce
from MyAlgorithm import MyAlgorithm



if __name__ == "__main__":

    cfg = config.load(sys.argv[1])

    #starting comm
    jdrc= comm.init(cfg, 'Stop')

    cameraC = jdrc.getCameraClient("Stop.CameraC")
    cameraL = jdrc.getCameraClient("Stop.CameraL")
    cameraR = jdrc.getCameraClient("Stop.CameraR")
    motors = jdrc.getMotorsClient ("Stop.Motors")
    pose3d = jdrc.getPose3dClient("Stop.Pose3D")
    
    algorithm=MyAlgorithm(pose3d, cameraC, cameraL, cameraR, motors)

    app = QApplication(sys.argv)
    myGUI = MainWindow()
    myGUI.setMotors(motors)
    myGUI.setCameraC(cameraC)
    myGUI.setCameraL(cameraL)
    myGUI.setCameraR(cameraR)
    myGUI.setPose3D(pose3d)
    myGUI.setAlgorithm(algorithm)
    myGUI.show()


    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()


    sys.exit(app.exec_())
