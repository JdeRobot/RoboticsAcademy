from gui.widgets.teleopWidget import TeleopWidget

__author__ = 'frivas'



from PyQt4 import QtGui,QtCore
from gui.form import Ui_MainWindow
from gui.widgets.cameraWidget import CameraWidget
from gui.widgets.logoWidget import LogoWidget

from jderobotTypes import CMDVel



class MainWindow(QtGui.QMainWindow, Ui_MainWindow):

    updGUI=QtCore.pyqtSignal()
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.teleop=TeleopWidget(self)
        self.tlLayout.addWidget(self.teleop)
        self.teleop.setVisible(True)
        
        self.logo = LogoWidget(self, self.logoLayout.parent().width(), self.logoLayout.parent().height())
        self.logoLayout.addWidget(self.logo)
        self.logo.setVisible(True)

        self.pushButton.clicked.connect(self.playClicked)
        self.pushButton.setCheckable(True)
        self.updGUI.connect(self.updateGUI)
        self.camera1=CameraWidget(self)

        self.stopButton.clicked.connect(self.stopClicked)

    def updateGUI(self):
        #print 'update gui'
        self.camera1.updateImage()
        #self.sensorsWidget.sensorsUpdate.emit()

    def getCameraL(self):
        return self.cameraL

    def setCameraL(self,camera):
        self.cameraL=camera

    def getCameraR(self):
        return self.cameraR

    def setCameraR(self,camera):
        self.cameraR=camera

    def getMotors(self):
        return self.motors

    def setMotors(self,motors):
        self.motors=motors

    def playClicked(self):
        if self.pushButton.isChecked():
            self.pushButton.setText('RUNNING')
            self.pushButton.setStyleSheet("background-color: green")
            self.algorithm.play()
        else:
            self.pushButton.setText('STOPPED')
            self.pushButton.setStyleSheet("background-color: red")
            self.algorithm.stop()

    def setAlgorithm(self, algorithm ):
        self.algorithm=algorithm

    def getAlgorithm(self):
        return self.algorithm

    def setXYValues(self,newX,newY):
        #print ("newX: %f, newY: %f" % (newX, newY) )

        vel = CMDVel()
        myW=-newX*self.motors.getMaxW()
        myV=-newY*self.motors.getMaxV()
        vel.vx = myV
        vel.az = myW
        self.motors.sendVelocities(vel)


    def stopClicked(self):
        vel = CMDVel()
        self.motors.sendVelocities(vel)
        self.teleop.returnToOrigin()

    def closeEvent(self, event):
        self.algorithm.kill()
        self.cameraR.stop()
        self.cameraL.stop()
        event.accept()
