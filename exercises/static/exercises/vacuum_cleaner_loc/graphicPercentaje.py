import sys, math
from math import pi as pi
import numpy as np
import cv2
from PyQt5.QtCore import QPoint, QRect, QSize, Qt, QPointF, QRectF, pyqtSignal, QTimer
from PyQt5.QtGui import (QBrush, QConicalGradient, QLinearGradient, QPainter, QPainterPath, QPalette, QPen, QPixmap, QPolygon, QRadialGradient, QColor, QTransform, QPolygonF, QKeySequence, QIcon)
from PyQt5.QtWidgets import (QApplication, QProgressBar, QCheckBox, QComboBox, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QSpinBox, QWidget, QPushButton, QSpacerItem, QSizePolicy, QLCDNumber)
from PyQt5 import QtGui, QtCore
from parallelIce.pose3dClient import Pose3DClient
import easyiceconfig as EasyIce
from gui.threadGUI import ThreadGUI
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

class MainWindow(QWidget):

    updGUI=pyqtSignal()
    def __init__(self, pose3d, parent=None):
        super(MainWindow, self).__init__(parent)
        
        layout = QGridLayout()
        
        self.seconds = 0
        self.MAXseconds = 900
        self.contSeconds = 0
        self.secondsArray = [0]
        
        self.devPercentajes = [0]
        self.percentaje = porcentajeWidget(self, pose3d)
        self.percentajePrev = 0
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        
        layout.addWidget(self.canvas)
        
        vSpacer = QSpacerItem(30, 50, QSizePolicy.Ignored, QSizePolicy.Ignored)
        layout.addItem(vSpacer,1,0)
        
        self.setFixedSize(1200,500);
        
        self.setLayout(layout)
        
        timer = QTimer(self)
        timer.start(1000)
        timer.timeout.connect(self.contadorTime)
        

    def contadorTime(self):
        if self.seconds < self.MAXseconds:
            self.percentaje.updateG()
            self.seconds += 1
            if self.seconds % 2 == 0:
                self.contSeconds += 1
                dif = float(float(self.percentaje.porcentajeCasa) - float(self.percentajePrev))
                self.devPercentajes.append(dif)
                self.secondsArray.append(self.contSeconds)
                self.percentajePrev = self.percentaje.porcentajeCasa
            
            ax = self.figure.add_subplot(111)
            ax.set_xlabel('Time')
            ax.set_ylabel('Percentage Derivative')
            ax.set_xlim([0, 450]);
            ax.set_ylim([0, 0.7]);
            ax.plot(self.secondsArray, self.devPercentajes,'r')
            self.canvas.draw()
            

class porcentajeWidget(QWidget):
    def __init__(self,winParent, pose3d):    
        super(porcentajeWidget, self).__init__()
        self.winParent=winParent
        self.map = cv2.imread("resources/images/mapgrannyannie.png", cv2.IMREAD_GRAYSCALE)
        self.map = cv2.resize(self.map, (500, 500))
        self.pose3d = pose3d
        self.porcentajeCasa = 0
        self.numPixels = self.calculatePixelsWhite()
        self.numPixelsRecorridos = 0
        self.numPixelInit = 656
        

    def RTx(self, angle, tx, ty, tz):
        RT = np.matrix([[1, 0, 0, tx], [0, math.cos(angle), -math.sin(angle), ty], [0, math.sin(angle), math.cos(angle), tz], [0,0,0,1]])
        return RT
        
    def RTy(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty], [-math.sin(angle), 0, math.cos(angle), tz], [0,0,0,1]])
        return RT
    
    def RTz(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), -math.sin(angle), 0, tx], [math.sin(angle), math.cos(angle),0, ty], [0, 0, 1, tz], [0,0,0,1]])
        return RT

    def RTVacuum(self):
        RTy = self.RTy(pi, 1, -1, 0)
        return RTy


    def calculatePixelsWhite(self):
        # Calculating the 100% of the pixels that can be traversed
        numPixels = 0
        for i in range(0, self.map.shape[1]):
            for j in range(0, self.map.shape[0]):
                if self.map[i][j] == 255:
                    numPixels = numPixels + 1
        return numPixels

    def calculatePercentaje(self):
        percentaje = float(self.numPixelsRecorridos * 100) / float(self.numPixels)
        # If vacuum is stopped, the percentaje is zero
        if self.numPixelsRecorridos == self.numPixelInit:
            percentaje = 0.0
        return percentaje


    def porcentajeRecorrido(self):
        x = self.pose3d.getX()
        y = self.pose3d.getY()
        scale = 50

        final_poses = self.RTVacuum() * np.matrix([[x], [y], [1], [1]]) * scale

        i_init = int(-50/4+final_poses.flat[0] + self.map.shape[1]/2)
        i_finish = int(50/4+final_poses.flat[0] + self.map.shape[1]/2)
        j_init = int(-50/4+final_poses[1] + self.map.shape[0]/2)
        j_finish = int(50/4+final_poses[1] + self.map.shape[0]/2)
        for k in range(i_init, i_finish+1):
            for l in range(j_init, j_finish+1):
                if (self.map[k][l] == 255):
                    self.numPixelsRecorridos = self.numPixelsRecorridos + 1
                    self.map[k][l] = 128

        self.porcentajeCasa = self.calculatePercentaje()


    def updateG(self):
        self.porcentajeRecorrido()
        self.update()  
        

if __name__ == "__main__":
    
    app = QApplication(sys.argv)
    ic = EasyIce.initialize(sys.argv)
    pose3d = Pose3DClient(ic, "Vacuum.Pose3D", True)

    myGUI = MainWindow(pose3d)
    myGUI.show()
    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()
    sys.exit(app.exec_())
