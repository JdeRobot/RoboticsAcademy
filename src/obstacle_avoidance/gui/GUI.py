from gui.widgets.teleopWidget import TeleopWidget
from gui.widgets.mapWidget import MapWidget

__author__ = 'frivas'



from PyQt4 import QtGui,QtCore
from gui.form import Ui_MainWindow
from gui.widgets.cameraWidget import CameraWidget



class MainWindow(QtGui.QMainWindow, Ui_MainWindow):

    updGUI=QtCore.pyqtSignal()
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.teleop=TeleopWidget(self)
        self.map=MapWidget(self)
        self.tlLayout.addWidget(self.teleop)
        self.teleop.setVisible(True)
        self.mapLayout.addWidget(self.map)
        self.map.setVisible(True)

        self.pushButton.clicked.connect(self.playClicked)
        self.pushButton.setCheckable(True)
        self.updGUI.connect(self.updateGUI)
        self.camera1=CameraWidget(self)

        self.stopButton.clicked.connect(self.stopClicked)

    def updateGUI(self):
        self.camera1.updateImage()
        (cx, cy) = self.algorithm.getCarDirection()
        (ox, oy) = self.algorithm.getObstaclesDirection()
        (ax, ay) = self.algorithm.getAverageDirection()
        (tx, ty) = self.algorithm.getCurrentTarget()
        self.map.setCarArrow(cx, cy)
        self.map.setObstaclesArrow(ox, oy)
        self.map.setAverageArrow(ax, ay)
        self.map.setTarget(tx, ty, self.sensor.getRobotX(), self.sensor.getRobotY(), self.sensor.getRobotTheta())
        self.map.setLaserValues(self.sensor.getLaserData())
        self.map.update()

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
