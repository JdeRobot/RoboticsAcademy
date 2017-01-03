from gui.widgets.teleopWidget import TeleopWidget

__author__ = 'frivas'



from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QMainWindow
from gui.form import Ui_MainWindow
from gui.widgets.cameraWidget import CameraWidget
from gui.widgets.logoWidget import LogoWidget
import numpy

class MainWindow(QMainWindow, Ui_MainWindow):

    updGUI=pyqtSignal()
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.teleop=TeleopWidget(self)
        self.tlLayout.addWidget(self.teleop)
        self.teleop.setVisible(True)
        self.pushButton.clicked.connect(self.playClicked)
        self.pushButton.setCheckable(True)
        self.updGUI.connect(self.updateGUI)
        self.camera1=CameraWidget(self)
        self.stopButton.clicked.connect(self.stopClicked)
        self.logo = LogoWidget(self)
        self.logoLayout.addWidget(self.logo)
        self.logo.setVisible(True)

    def updateGUI(self):
        #print ('update gui')
        self.camera1.updateImage()
        #self.sensorsWidget.sensorsUpdate.emit()
        #self.glWidget.update()

    def getSensor(self):
        return self.sensor

    def setSensor(self,sensor):
        self.sensor=sensor

    def playClicked(self):
        self.sensor.setPlayButton(self.pushButton.isChecked())
        if self.pushButton.isChecked():
            self.pushButton.setText('RUNNING')
            self.pushButton.setStyleSheet("background-color: green")
        else:
            self.pushButton.setText('STOPPED')
            self.pushButton.setStyleSheet("background-color: red")

    def setAlgorithm(self, algorithm ):
        self.algorithm=algorithm

    def getAlgorithm(self):
        return self.algorithm

    def setXYValues(self,newX,newY):
        self.sensor.setV(-newY)
        self.sensor.setW(newX)

    def stopClicked(self):
        self.sensor.setV(0)
        self.sensor.setW(0)
        self.teleop.returnToOrigin()



