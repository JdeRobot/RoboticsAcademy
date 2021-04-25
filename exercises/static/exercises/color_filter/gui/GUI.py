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


from PyQt5.QtCore import pyqtSignal, Qt,QSize
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QWidget, QLabel, QHBoxLayout, QVBoxLayout, QGroupBox, QCheckBox
from gui.ui_gui import Ui_MainWindow
from PyQt5.QtGui import QImage, QPixmap
from gui.communicator import Communicator

from gui.logoWidget import LogoWidget


class MainWindow(QWidget, Ui_MainWindow):

    
    updGUI=pyqtSignal()
    def __init__(self, camera, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUI(self)

        self.camera = camera
        
        
        
        self.updGUI.connect(self.updateImage)
        self.updGUI.connect(self.updateCamImage)

        self.cameraCommunicator=Communicator()
        self.colorFilterCommunicator=Communicator()
        self.trackingCommunicator = Communicator()

        self.playButton.clicked.connect(self.playClicked)
        self.playButton.setCheckable(True)


