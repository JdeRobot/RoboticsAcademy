from gui.widgets.teleopWidget import TeleopWidget

__author__ = 'frivas'


import resources_rc
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QMainWindow
from gui.form import Ui_MainWindow
from gui.widgets.cameraWidget import CameraWidget
from gui.widgets.logoWidget import LogoWidget
from gui.widgets.mapWidget import MapWidget
from gui.widgets.chronoWidget import ChronoWidget


class MainWindow(QMainWindow, Ui_MainWindow):

    updGUI=pyqtSignal()
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        # self.setStyleSheet(open("gui/style.qss", "r").read())
        #self.teleop=TeleopWidget(self)
        #self.tlLayout.addWidget(self.teleop)
        #self.teleop.setVisible(True)
        #self.logo = LogoWidget(self)
        #self.logoLayout.addWidget(self.logo)
        #self.logo.setVisible(True)

        self.pushButton.clicked.connect(self.playClicked)
        self.pushButton.setCheckable(True)
        self.updGUI.connect(self.updateGUI)
        self.camera1=CameraWidget(self)
        self.mapW = MapWidget(self)
        self.verticalLayoutMap.addWidget(self.mapW)
        self.chronoW = ChronoWidget(self)
        self.verticalLayoutChrono.addWidget(self.chronoW)
        print(self.mapW.width(), self.mapW.height())

        self.stopButton.clicked.connect(self.stopClicked)

    def updateGUI(self):
        #print 'update gui'
        cx = self.pose3d.getPose3d().x
        cy = self.pose3d.getPose3d().y
        phx, phy = self.algorithm.synchronize()
        self.mapW.setCarPos(cx, cy)
        self.mapW.setPhantomPos(phx, phy)
        duration = self.algorithm.get_duration()
        initime = self.algorithm.get_initime()
        self.chronoW.setTime(initime, duration)
        self.camera1.updateImage()

    def getCamera(self):
        return self.camera

    def setCamera(self,camera):
        self.camera=camera

    def getMotors(self):
        return self.motors

    def setMotors(self,motors):
        self.motors=motors

    def getPose3D(self):
        return self.pose3d

    def setPose3D(self,pose3d, pose3dphantom):
        self.pose3dphantom = pose3dphantom
        self.pose3d=pose3d

    def playClicked(self):
        if self.pushButton.isChecked():
            self.pushButton.setText('Stop Code')
            self.pushButton.setStyleSheet("background-color: #ec7063")
            self.algorithm.play()
        else:
            self.pushButton.setText('Play Code')
            self.pushButton.setStyleSheet("background-color: #7dcea0")
            self.algorithm.stop()

    def setAlgorithm(self, algorithm ):
        self.algorithm=algorithm

    def getAlgorithm(self):
        return self.algorithm

    def setXYValues(self,newX,newY):
        #print ("newX: %f, newY: %f" % (newX, newY) )
        myW=-newX*self.motors.getMaxW()
        myV=-newY*self.motors.getMaxV()
        self.motors.sendV(myV)
        self.motors.sendW(myW)

    def stopClicked(self):
        self.motors.sendV(0)
        self.motors.sendW(0)
        #self.teleop.returnToOrigin()

    def closeEvent(self, event):
        self.algorithm.kill()
        self.camera.stop()
        event.accept()
