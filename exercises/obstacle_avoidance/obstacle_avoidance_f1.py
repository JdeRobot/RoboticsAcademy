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
#  Authors:
#       Eduardo Perdices <eperdices@gsyc.es>
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#  Adapted to Kobuki/Turtlebot by:
#       Julio Vega <julio.vega@urjc.es>
#

import sys, os
import config
from PyQt5.QtWidgets import QApplication
from gui.GUI import MainWindow
from gui.threadGUI import ThreadGUI
from MyAlgorithm_f1 import MyAlgorithm

from interfaces.camera import ListenerCamera
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser
from interfaces.motors import PublisherMotors

if __name__ == "__main__":
	cfg = config.load(sys.argv[1])
		
	cam_path = cfg.getProperty("ObstacleAvoidance.CameraPath")
	mot_path = cfg.getProperty("ObstacleAvoidance.MotorsPath")
	odo_path = cfg.getProperty("ObstacleAvoidance.Pose3DPath")
	las_path = cfg.getProperty("ObstacleAvoidance.LaserPath")

	camera = ListenerCamera(cam_path)
	motors = PublisherMotors(mot_path, 4, 0.3)
	pose3d = ListenerPose3d(odo_path)
	laser = ListenerLaser(las_path)

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

