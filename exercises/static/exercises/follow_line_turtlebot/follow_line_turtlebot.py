#!/usr/bin/python3
#
#  Copyright (C) 1997-2019 JDE Developers Team
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
#  Adapted to turtlebot by:
#       Julio Vega <julio.vega@urjc.es>

# General imports
import sys
import config

# Practice imports
from gui.GUI import MainWindow
from gui.threadGUI import ThreadGUI
from MyAlgorithm import MyAlgorithm
from PyQt5.QtWidgets import QApplication
from interfaces.camera import ListenerCamera
from interfaces.motors import PublisherMotors

if __name__ == "__main__":
    cfg = config.load(sys.argv[1])
    #jdrc= comm.init(cfg, 'FollowLineTurtlebot')

    robot = cfg.getProperty("FollowLineTurtlebot.Robot")

    maxv = cfg.getPropertyWithDefault("FollowLineTurtlebot.MaxV", 2)
    maxw = cfg.getPropertyWithDefault("FollowLineTurtlebot.MaxV", 0.3)

    if (robot == "simkobuki"):
        cam_path = cfg.getProperty("FollowLineTurtlebot.SimCameraPath")
        mot_path = cfg.getProperty("FollowLineTurtlebot.SimMotorsPath")

    else: # realkobuki
        cam_path = cfg.getProperty("FollowLineTurtlebot.RealCameraPath")
        mot_path = cfg.getProperty("FollowLineTurtlebot.RealMotorsPath")

    camera = ListenerCamera(cam_path)
    motors = PublisherMotors(mot_path, maxv, maxw)

    algorithm=MyAlgorithm(camera, motors)

    app = QApplication(sys.argv)
    myGUI = MainWindow()
    myGUI.setCamera(camera)
    myGUI.setMotors(motors)
    myGUI.setAlgorithm(algorithm)
    myGUI.show()

    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()

    sys.exit(app.exec_())
