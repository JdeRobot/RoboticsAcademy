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
#       Carlos Awadallah Estevez<carlosawadallah@gmail.com>
#       Alberto Martin Florido <almartinflorido@gmail.com>
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#


from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow
from gui.ui_gui import Ui_MainWindow
from gui.teleopWidget import TeleopWidget
from gui.communicator import Communicator

from gui.segmentWidget import SegmentWidget

from gui.logoWidget import LogoWidget

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

        self.record = False

        self.updGUI.connect(self.updateGUI)
        
        self.segmentCheck.stateChanged.connect(self.showSegmentWidget)
        
        self.segmentWidget=SegmentWidget(self)

        self.segmentCommunicator=Communicator()
        self.trackingCommunicator = Communicator()

        #self.stopButton.clicked.connect(self.stopClicked)
        self.playButton.clicked.connect(self.playClicked)
        self.playButton.setCheckable(True)
        self.resetButton.clicked.connect(self.resetClicked)
        self.takeoff=False
        self.reset=False
      
    def setCamera(self, camera):
        self.camera = camera
        #self.cameraWidget.show() #uncomment if wanted to see the image of the 
                                  #camera whenever you start the execution

    def getCamera(self):
        return self.camera

    def setMotors(self, motors):
        self.motors = motors

    def getMotors(self):
		return self.motors

    def setAlgorithm(self, algorithm ):
        self.algorithm=algorithm

    def getAlgorithm(self):
        return self.algorithm
    
    def updateGUI(self):
        self.segmentWidget.imageUpdate.emit()
    
    def playClicked(self):
        if self.playButton.isChecked():
            self.segmentWidget.show()
            self.segmentCheck.setChecked(True)
            icon = QtGui.QIcon()
            self.playButton.setText("Stop Code")
            self.playButton.setStyleSheet("background-color: #ec7063")
            icon.addPixmap(QtGui.QPixmap(":/images/stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.playButton.setIcon(icon)
            self.algorithm.play()
        else:
            icon = QtGui.QIcon()
            self.playButton.setStyleSheet("background-color: #7dcea0")
            icon.addPixmap(QtGui.QPixmap(":/images/play.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.playButton.setIcon(icon)
            self.playButton.setText("Play Code")
            self.algorithm.stop()
            self.teleop.stopSIG.emit()

    
    #def stopClicked(self):        
     #   self.algorithm.stop()
     #   self.teleop.stopSIG.emit()
        # ponerrrrr: self.teleop.returnToOrigin()

    def resetClicked(self):
        if self.reset == True:
            self.resetButton.setText("Reset")
            self.reset=False
        else:
            self.resetButton.setText("Unreset")
            self.reset=True

    def showSegmentWidget(self,state):
        if state == Qt.Checked:
            self.segmentWidget.show()
        else:
            self.segmentWidget.close()
            
    def closeSegmentWidget(self):
        self.segmentCheck.setChecked(False)

    def setXYValues(self, newX, newY):
        limits = self.motors.getLimits()
        pan =  newX*limits.maxPan
        tilt = - newY*limits.maxTilt

        self.YValue.setText(str(tilt))
        self.XValue.setText(str(pan))
        #self.YValue.setText("{:.0f}".format(tilt))
        #self.XValue.setText("{:.0f}".format(pan))
        if (self.motors):
            self.motors.setPTMotorsData(pan, tilt, limits.maxPanSpeed, limits.maxTiltSpeed)

    def closeEvent(self, event):
        self.algorithm.kill()
        self.camera.client.stop()
        self.closeSegmentWidget()
        event.accept()


