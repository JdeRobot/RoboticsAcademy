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
#       Alberto Martin Florido <almartinflorido@gmail.com>
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#

import sys
from MyAlgorithm import MyAlgorithm
from gui.threadGUI import ThreadGUI
from gui.GUI import MainWindow
from PyQt5.QtWidgets import QApplication

from drone import Drone

import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)

if __name__ == '__main__':

    drone = Drone("mavros/cmd/arming", "mavros/cmd/land","mavros/set_mode",  "/mavros/setpoint_velocity/cmd_vel","/mavros/local_position/odom",  "/solo/cam_frontal/image_raw")


    algorithm=MyAlgorithm(drone)


    app = QApplication(sys.argv)
    frame = MainWindow()
    frame.setDrone(drone)
    frame.setAlgorithm(algorithm)
    frame.show()



    t2 = ThreadGUI(frame)
    t2.daemon=True
    t2.start()

    sys.exit(app.exec_())
