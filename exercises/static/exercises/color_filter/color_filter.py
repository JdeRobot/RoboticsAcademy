#!/usr/bin/python3.5
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
#       Carlos Awadallah Estevez <carlosawadallah@gmail.com>
#

import sys
import yaml
import comm
import config

from Camera.cameraSegment import CameraSegment

from MyAlgorithm import MyAlgorithm
from gui.threadGUI import ThreadGUI
from gui.GUI import MainWindow
from PyQt5.QtWidgets import QApplication

import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)


def getCamera(cfg):
    cfg = config.load(sys.argv[1])
    jdrc = comm.init(cfg, 'Color_Filter')
    proxy = jdrc.getCameraClient('Color_Filter')
    from Camera.cameraSegment import CameraSegment
    cam = CameraSegment(proxy)
    return cam 


if __name__ == '__main__':

    cfg = config.load(sys.argv[1])
    #print(cfg)
    jdrc= comm.init(cfg, 'Color_Filter')

    cameraCli = jdrc.getCameraClient("Color_Filter.Camera")
    camera = CameraSegment(cameraCli)

    # Threading the camera...
    algorithm=MyAlgorithm(camera)

    app = QApplication(sys.argv)
    frame = MainWindow(camera)
    # frame.setCamera(camera)
    frame.setAlgorithm(algorithm)
    frame.show()

    t2 = ThreadGUI(frame)  
    t2.daemon=True
    t2.start()
    
    sys.exit(app.exec_()) 
