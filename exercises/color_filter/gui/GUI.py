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


from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow
from gui.ui_gui import Ui_MainWindow
from gui.cameraWidget import CameraWidget
from gui.communicator import Communicator
from gui.colorFilterWidget import  ColorFilterWidget
from gui.logoWidget import LogoWidget

class MainWindow(QMainWindow, Ui_MainWindow):
    
    updGUI=pyqtSignal()
    def __init__(self, camera, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)

        self.camera = camera

        self.logo = LogoWidget(self, self.logoLayout.parent().width(), self.logoLayout.parent().height())
        self.logoLayout.addWidget(self.logo)
        self.logo.setVisible(True)


        self.updGUI.connect(self.updateGUI)
        
        self.cameraCheck.stateChanged.connect(self.showCameraWidget)
        self.colorFilterCheck.stateChanged.connect(self.showColorFilterWidget)
        
        self.cameraWidget=CameraWidget(self)
        self.colorFilterWidget=ColorFilterWidget(self)

        self.cameraCommunicator=Communicator()
        self.colorFilterCommunicator=Communicator()
        self.trackingCommunicator = Communicator()

        self.playButton.clicked.connect(self.playClicked)
        self.playButton.setCheckable(True)
      
    def getCamera(self):
        return self.camera

    def setCamera(self,camera):
        self.camera = camera


    def setAlgorithm(self, algorithm ):
        self.algorithm=algorithm

    def getAlgorithm(self):
        return self.algorithm
    
    def updateGUI(self):
        self.cameraWidget.imageUpdate.emit()
        self.colorFilterWidget.imageUpdate.emit()


    def playClicked(self):
        if self.playButton.isChecked():
            icon = QtGui.QIcon()
            self.playButton.setText("Stop Code")
            self.playButton.setStyleSheet("background-color: #ec7063")
            icon.addPixmap(QtGui.QPixmap(":/images/stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.playButton.setIcon(icon)
            self.algorithm.play()
        else:
            icon = QtGui.QIcon()
            icon.addPixmap(QtGui.QPixmap(":/images/play.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.playButton.setIcon(icon)
            self.playButton.setText("Play Code")
            self.playButton.setStyleSheet("background-color: #7dcea0")
            self.algorithm.stop()
    
        
    def showCameraWidget(self,state):
        if state == Qt.Checked:
            self.cameraWidget.show()
        else:
            self.cameraWidget.close()
            
    def closeCameraWidget(self):
        self.cameraCheck.setChecked(False)

    def showColorFilterWidget(self,state):
        if state == Qt.Checked:
            self.colorFilterWidget.show()
        else:
            self.colorFilterWidget.close()

    def closeColorFilterWidget(self):
        self.colorFilterCheck.setChecked(False)
    

    def closeEvent(self, event):
        self.algorithm.kill()
        self.colorFilterWidget.closeEvent(event)
        self.closeColorFilterWidget()
        self.closeCameraWidget()
        self.camera.client.stop()
        event.accept()

