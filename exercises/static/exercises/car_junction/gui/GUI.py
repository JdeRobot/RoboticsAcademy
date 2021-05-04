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
#       Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>
#
from PyQt5 import QtCore, QtGui, QtWidgets
from gui.widgets.teleopWidget import TeleopWidget
from gui.widgets.mapWidget import MapWidget
from gui.widgets.mapWidget import LogoWidget
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QMainWindow
from gui.form import Ui_MainWindow
from gui.widgets.cameraWidget import CameraWidget


class MainWindow(QMainWindow, Ui_MainWindow):

    updGUI = pyqtSignal()

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.teleop = TeleopWidget(self)
        self.logo = LogoWidget(self)
        self.tlLayout.addWidget(self.teleop)
        self.teleop.setVisible(True)
        self.logoLayout.addWidget(self.logo)
        self.logo.setVisible(True)

        self.pushButton.clicked.connect(self.playClicked)
        self.pushButton.setCheckable(True)
        self.updGUI.connect(self.updateGUI)
        self.cameraW = CameraWidget(self)

        self.resetButton.clicked.connect(self.resetClicked)

    def updateGUI(self):
        self.cameraW.updateImage()

    def getPose3D(self):
        return self.pose3d

    def setPose3D(self, pose3d):
        self.pose3d = pose3d

    def getCameraC(self):
        return self.cameraC

    def setCameraC(self, camera):
        self.cameraC = camera

    def getCameraL(self):
        return self.cameraL

    def setCameraL(self, camera):
        self.cameraL = camera

    def getCameraR(self):
        return self.cameraR

    def setCameraR(self, camera):
        self.cameraR = camera

    def getMotors(self):
        return self.motors

    def setMotors(self, motors):
        self.motors = motors

    def playClicked(self):
        if self.pushButton.isChecked():
            icon = QtGui.QIcon()
            self.pushButton.setText("Stop Code")
            self.pushButton.setStyleSheet("background-color: #ec7063")
            icon.addPixmap(QtGui.QPixmap(":/images/stop.png"),
                           QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.pushButton.setIcon(icon)
            self.algorithm.play()
        else:
            icon = QtGui.QIcon()
            self.pushButton.setStyleSheet("background-color: #7dcea0")
            icon.addPixmap(QtGui.QPixmap(":/images/play.png"),
                           QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.pushButton.setIcon(icon)
            self.pushButton.setText("Play Code")
            self.algorithm.stop()

    def setAlgorithm(self, algorithm):
        self.algorithm = algorithm

    def getAlgorithm(self):
        return self.algorithm

    def setXYValues(self, newX, newY):
        myW = -newX*self.motors.getMaxW()
        myV = -newY*self.motors.getMaxV()
        self.motors.sendV(myV)
        self.motors.sendW(myW)
        # self.motors.sendVelocities()

    def resetClicked(self):
        self.motors.sendV(0)
        self.motors.sendW(0)
        # self.motors.sendVelocities()
        self.teleop.returnToOrigin()

    def closeEvent(self, event):
        self.algorithm.kill()
        self.cameraL.stop()
        self.cameraR.stop()
        self.cameraC.stop()
        event.accept()
