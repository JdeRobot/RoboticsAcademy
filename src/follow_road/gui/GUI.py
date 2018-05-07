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
from PyQt5.QtWidgets import QMainWindow
from gui.ui_gui import Ui_MainWindow
from gui.teleopWidget import TeleopWidget
from gui.cameraWidget import CameraWidget
from gui.communicator import Communicator
from gui.sensorsWidget import SensorsWidget
from gui.logoWidget import LogoWidget
from PyQt5 import QtCore, QtGui, QtWidgets

class MainWindow(QMainWindow, Ui_MainWindow):

    updGUI=pyqtSignal()
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.teleop=TeleopWidget(self)
        self.tlLayout.addWidget(self.teleop)
        self.teleop.setVisible(True)
        self.logo = LogoWidget(self, self.logoLayout.parent().width(), self.logoLayout.parent().height())
        self.logoLayout.addWidget(self.logo)
        self.logo.setVisible(True)

        self.updGUI.connect(self.updateGUI)
        self.sensorsCheck.stateChanged.connect(self.showSensorsWidget)
        self.sensorsWidget=SensorsWidget(self)

        self.rotationDial.valueChanged.connect(self.rotationChange)
        self.altdSlider.valueChanged.connect(self.altitudeChange)
        self.changeCamButton.clicked.connect(self.changeCamera)
        self.pushButton.clicked.connect(self.pushClicked)
        self.pushButton.setCheckable(True)
        self.resetButton.clicked.connect(self.resetClicked)
        self.takeoffButton.clicked.connect(self.takeOffClicked)
        self.camera1=CameraWidget(self)

        self.record = False
        self.takeoff=False
        self.reset=False

    def getCamera(self):
        return self.camera

    def setCamera(self,camera):
        self.camera = camera

    def getNavData(self):
        return self.navdata

    def setNavData(self,navdata):
        self.navdata = navdata

    def getPose3D(self):
        return self.pose

    def setPose3D(self,pose):
        self.pose = pose

    def getCMDVel(self):
        return self.cmdvel

    def setCMDVel(self,cmdvel):
        self.cmdvel = cmdvel

    def getExtra(self):
        return self.extra

    def setExtra(self,extra):
        self.extra = extra

    def setAlgorithm(self, algorithm ):
        self.algorithm=algorithm

    def getAlgorithm(self):
        return self.algorithm

    def updateGUI(self):
        self.camera1.updateImage()
        self.sensorsWidget.sensorsUpdate.emit()

    def pushClicked(self):
        if self.pushButton.isChecked():
            icon = QtGui.QIcon()
            self.pushButton.setText('Stop Code')
            self.pushButton.setStyleSheet("background-color: #ec7063")
            icon.addPixmap(QtGui.QPixmap(":/images/stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.pushButton.setIcon(icon)
            self.algorithm.play()
        else:
            icon = QtGui.QIcon()
            self.pushButton.setText('Play Code')
            self.pushButton.setStyleSheet("background-color: #7dcea0")
            icon.addPixmap(QtGui.QPixmap(":/images/play.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.pushButton.setIcon(icon)
            self.algorithm.stop()

    def resetClicked(self):
        if self.reset == True:
            self.resetButton.setText("Reset")
            self.extra.reset()
            self.reset=False
        else:
            self.resetButton.setText("Unreset")
            self.extra.reset()
            self.reset=True

    def takeOffClicked(self):
        if(self.takeoff==True):
            self.takeoffButton.setText("Take Off")
            self.extra.land()
            self.takeoff=False
        else:
            self.takeoffButton.setText("Land")
            self.extra.takeoff()
            self.takeoff=True

    def resetClicked(self):
        if self.reset == True:
            self.resetButton.setText("Reset")
            self.extra.reset()
            self.reset=False
        else:
            self.resetButton.setText("Unreset")
            self.extra.reset()
            self.reset=True

    def changeCamera(self):
        self.extra.toggleCam()

    def showSensorsWidget(self,state):
        if state == Qt.Checked:
            self.sensorsWidget.show()
        else:
            self.sensorsWidget.close()

    def closeSensorsWidget(self):
        self.sensorsCheck.setChecked(False)

    def rotationChange(self,value):
        value=(1.0/(self.rotationDial.maximum()/2))*(value - (self.rotationDial.maximum()/2))
        self.rotValue.setText('%.2f' % value)
        self.cmdvel.setYaw(value)
        self.cmdvel.sendVelocities()

    def altitudeChange(self,value):
        value=(1.0/(self.altdSlider.maximum()/2))*(value - (self.altdSlider.maximum()/2))
        self.rotValue.setText('%.2f' % value)
        self.cmdvel.setVZ(value)
        self.cmdvel.sendVelocities()

    def setXYValues(self,newX,newY):
        self.XValue.setText('%.2f' % newX)
        self.YValue.setText('%.2f' % newY)
        self.cmdvel.setVX(-newY)
        self.cmdvel.setVY(-newX)
        self.cmdvel.sendVelocities()

    def closeEvent(self, event):
        self.algorithm.kill()
        self.camera.stop()
        self.navdata.stop()
        self.pose.stop()
        event.accept()
