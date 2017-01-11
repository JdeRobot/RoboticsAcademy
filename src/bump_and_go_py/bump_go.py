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
#


import sys
from gui.gui import Window
from gui.threadGUI import ThreadGUI
from gui.machine import Machine
from parallelIce.laserClient import LaserClient
from parallelIce.pose3dClient import Pose3DClient
from parallelIce.motors import Motors
import easyiceconfig as EasyIce
from MyAlgorithm import MyAlgorithm
from PyQt5.QtWidgets import QApplication




if __name__ == "__main__":

	machine = Machine(3)
	machine.setStateName(0, 'Forward') 
	machine.setStateName(1, 'Backward')
	machine.setStateName(2, 'Turn')
	machine.addTransition(0, 1,'obstacle < th')
	machine.addTransition(1, 2,'obstacle > th+x')
	machine.addTransition(2, 0,'turn x rads')
	machine.addTransition(2, 1,'')


	ic = EasyIce.initialize(sys.argv)
	pose3d = Pose3DClient(ic, 'BumpGo.Pose3D')
	laser = LaserClient(ic, 'BumpGo.Laser')
	motors = Motors (ic, 'BumpGo.Motors')
	laser.start()
	pose3d.start()
	algorithm=MyAlgorithm(laser, motors, pose3d, machine)

	app = QApplication(sys.argv)
	myGUI = Window(machine, algorithm)
	myGUI.show()

	t2 = ThreadGUI(myGUI)
	t2.daemon=True
	t2.start()

	sys.exit(app.exec_())

