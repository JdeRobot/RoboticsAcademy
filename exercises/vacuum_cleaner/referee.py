import sys, math
import comm
import config
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
import rospy
import threading
from math import asin, atan2, pi
from jderobotTypes import Pose3d
from nav_msgs.msg import Odometry

class MainWindow(QWidget):

    updGUI=pyqtSignal()
    def __init__(self, pose3d, parent=None):
        super(MainWindow, self).__init__(parent)

        layout = QGridLayout()
        self.Percentage = PercentageWidget(self, pose3d)
        self.timeDigital = timeDigitalWidget(self, self.Percentage)
        self.timeAnalog = timeAnalogWidget(self)
        self.map = mapWidget(self, pose3d)
        self.logo = logoWidget(self)
        layout.addWidget(self.timeDigital,0,2)
        layout.addWidget(self.Percentage,0,0)
        layout.addWidget(self.map,1,0)
        layout.addWidget(self.timeAnalog,1,2)
        layout.addWidget(self.logo,2,2)

        vSpacer = QSpacerItem(50, 50, QSizePolicy.Ignored, QSizePolicy.Ignored)
        layout.addItem(vSpacer,1,0)

        self.setFixedSize(840,640);

        self.setLayout(layout)
        self.updGUI.connect(self.update)

    def update(self):
        self.Percentage.updateG()
        self.map.updateG()
        self.timeAnalog.updateG()


class logoWidget(QWidget):
    def __init__(self, winParent):
        super(logoWidget, self).__init__()
        self.winParent=winParent
        self.logo = cv2.imread("resources/logo_jderobot1.png", cv2.IMREAD_UNCHANGED)
        self.logo = cv2.resize(self.logo, (100, 100))
        image = QtGui.QImage(self.logo.data, self.logo.shape[1], self.logo.shape[0], QtGui.QImage.Format_ARGB32);
        self.pixmap = QtGui.QPixmap.fromImage(image)
        self.height = self.pixmap.height()
        self.width = self.pixmap.width()
        self.mapWidget = QLabel(self)
        self.mapWidget.setPixmap(self.pixmap)
        self.mapWidget.resize(self.width, self.height)
        self.setMinimumSize(100,100)


class mapWidget(QWidget):
    def __init__(self,winParent, pose3d):
        super(mapWidget, self).__init__()
        self.winParent=winParent
        self.map = cv2.imread("resources/images/mapgrannyannie.png", cv2.IMREAD_GRAYSCALE)
        self.map = cv2.resize(self.map, (500, 500))
        image = QtGui.QImage(self.map.data, self.map.shape[1], self.map.shape[0], self.map.shape[1], QtGui.QImage.Format_Indexed8);
        self.pixmap = QtGui.QPixmap.fromImage(image)
        self.height = self.pixmap.height()
        self.width = self.pixmap.width()
        self.mapWidget = QLabel(self)
        self.mapWidget.setPixmap(self.pixmap)
        self.mapWidget.resize(self.width, self.height)

        self.resize(100,100)
        self.setMinimumSize(500,500)

        self.pose3d = pose3d
        self.trail = []


    def RTy(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty], [-math.sin(angle), 0, math.cos(angle), tz], [0,0,0,1]])
        return RT

    def RTVacuum(self):
        RTy = self.RTy(pi, 0.6, -1, 0)
        return RTy


    def drawCircle(self, painter, centerX, centerY):
        yaw = self.pose3d.getPose3d().yaw
        pen = QPen(Qt.blue, 2)
        painter.setPen(pen)
        brush = QtGui.QBrush(QtCore.Qt.SolidPattern)
        brush.setColor(QtGui.QColor(Qt.blue))
        painter.setBrush(brush)
        painter.drawEllipse(centerX, centerY, 50/3, 50/3)


    def drawTrail(self, painter):
        x = self.pose3d.getPose3d().x
        y = self.pose3d.getPose3d().y
        scale = 50

        final_poses = self.RTVacuum() * np.matrix([[x], [y], [1], [1]]) * scale

        # Vacuum's way
        self.trail.append([final_poses.flat[0], final_poses.flat[1]])

        for i in range(0, len(self.trail)):
            self.drawCircle(painter, self.trail[i][0], self.trail[i][1])


    def paintEvent(self, event):
        copy = self.pixmap.copy()
        painter = QtGui.QPainter(copy)
        painter.translate(QPoint(self.width/2, self.height/2))
        self.drawTrail(painter)
        self.mapWidget.setPixmap(copy)
        painter.end()

    def updateG(self):
        self.update()



class PercentageWidget(QWidget):
    def __init__(self,winParent, pose3d):
        super(PercentageWidget, self).__init__()
        self.winParent=winParent
        self.map = cv2.imread("resources/images/mapgrannyannie.png", cv2.IMREAD_GRAYSCALE)
        self.map = cv2.resize(self.map, (500, 500))
        image = QtGui.QImage(self.map.data, self.map.shape[1], self.map.shape[0], self.map.shape[1], QtGui.QImage.Format_Indexed8);
        self.pixmap = QtGui.QPixmap.fromImage(image)
        self.pose3d = pose3d
        self.PercentageHouse = 0
        self.numPixels = self.calculatePixelsWhite()
        self.numPixelsWalked = 0

        vLayout = QVBoxLayout()

        self.PercentageWalked()

        self.Percentage = QLabel("Percentage: " + str(round(self.PercentageHouse, 3)) + ' %')

        vLayout.addWidget(self.Percentage, 0)

        self.bar = QProgressBar()
        self.bar.setValue(self.PercentageHouse)
        st = "QProgressBar::chunk {background-color: #ff0000;}\n QProgressBar {border: 1px solid grey;border-radius: 2px;text-align: center;background: #eeeeee;}"
        self.bar.setStyleSheet(st)
        self.bar.setTextVisible(False)
        vLayout.addWidget(self.Percentage, 0)
        vLayout.addWidget(self.bar, 0)

        vSpacer = QSpacerItem(30, 80, QSizePolicy.Ignored, QSizePolicy.Ignored)
        vLayout.addItem(vSpacer)

        self.setLayout(vLayout)



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
        img = self.pixmap.toImage()
        for i in range(0, self.map.shape[1]):
            for j in range(0, self.map.shape[0]):
                c = img.pixel(i,j)
                color = QtGui.QColor(c).getRgbF()
                if color == (1.0, 1.0, 1.0, 1.0):
                    numPixels = numPixels + 1
        return numPixels

    def calculatePercentage(self):
        Percentage = self.numPixelsWalked * 100 / self.numPixels
        return Percentage


    def PercentageWalked(self):
        x = self.pose3d.getPose3d().x
        y = self.pose3d.getPose3d().y
        scale = 50

        img = self.pixmap.toImage()
        final_poses = self.RTVacuum() * np.matrix([[x], [y], [1], [1]]) * scale

        i_init = int(-50/4+final_poses.flat[0] + self.map.shape[1]/2)
        i_finish = int(50/4+final_poses.flat[0] + self.map.shape[1]/2)
        j_init = int(-50/4+final_poses[1] + self.map.shape[0]/2)
        j_finish = int(50/4+final_poses[1] + self.map.shape[0]/2)
        for k in range(i_init, i_finish+1):
            for l in range(j_init, j_finish+1):
                c = img.pixel(k,l)
                color = QtGui.QColor(c).getRgbF()
                if color == (1.0, 1.0, 1.0, 1.0):
                    if self.map[k][l] != 128:
                        self.numPixelsWalked = self.numPixelsWalked + 1
                        self.map[k][l] = 128
        self.PercentageHouse = self.calculatePercentage()


    def updateG(self):
        self.PercentageWalked()
        self.Percentage.setText("Percentage: " + str(round(self.PercentageHouse, 3)) + ' %')
        self.bar.setValue(self.PercentageHouse)
        self.update()


class timeDigitalWidget(QWidget):

    time = pyqtSignal()
    def __init__(self,winParent, Percentage):
        super(timeDigitalWidget, self).__init__()
        self.winParent=winParent
        self.seconds = 900
        self.pose3d = pose3d
        self.Percentage = Percentage
        self.show = False
        self.MAX_PERCENT = 30
        self.MAX_MARK = 10

        self.hLayout = QHBoxLayout()

        timeLabel = QLabel("Time")
        self.lcd = QLCDNumber(self)
        self.lcd.setMaximumSize(100,50)
        self.hLayout.addWidget(timeLabel,0)
        self.hLayout.addWidget(self.lcd, 1)

        hSpacer = QSpacerItem(300, 30, QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.hLayout.addItem(hSpacer)

        self.setLayout(self.hLayout)

        timer = QTimer(self)
        timer.start(1000)
        timer.timeout.connect(self.printTime)

        # get the palette
        palette = self.lcd.palette()

        # foreground color
        palette.setColor(palette.WindowText, QColor(85, 85, 255))
        # background color
        palette.setColor(palette.Background, QColor(0, 170, 255))
        # "light" border
        palette.setColor(palette.Light, QColor(255, 0, 0))
        # "dark" border
        palette.setColor(palette.Dark, QColor(0, 255, 0))

        # set the palette
        self.lcd.setPalette(palette)

    def showMark(self):
        self.show = True
        mark = self.testPercentage()
        markLabel = QLabel('Final mark: ' + str(mark))
        self.hLayout.addWidget(markLabel, 0)
        self.setLayout(self.hLayout)

    def printTime(self):

        if self.seconds > 0:
            self.seconds -= 1
        else:
            if not self.show:
                self.showMark()
        self.lcd.display(self.seconds)


    def testPercentage(self):
        pHouse = self.Percentage.calculatePercentage()
        markPerc = float(pHouse) * float(self.MAX_MARK) / float(self.MAX_PERCENT)
        if pHouse > self.MAX_PERCENT:
            markPerc = 10
        return markPerc


class timeAnalogWidget(QWidget):

    time = pyqtSignal()
    def __init__(self,winParent):
        super(timeAnalogWidget, self).__init__()
        self.winParent=winParent
        self.rectangle = QRectF(0.0, 0.0, 300.0, 300.0)
        self.angle = -pi/2
        self.angleMinutes = -pi/2
        self.seconds = 900
        self.accountant = 0
        self.minutes = 0

        timer = QTimer(self)
        timer.start(1000)
        timer.timeout.connect(self.accountantTime)

    def drawWhiteZones(self, painter):
        self.setStyle(painter, QColor(255,255,255),QColor(255,255,255),1)
        startAngle = 0 * 16
        spanAngle = 360 * 16
        painter.drawPie(self.rectangle, startAngle, spanAngle)

    def drawCLockLines(self, painter):
        radius = 130
        angle = -pi/2
        for i in range (0, 60):
            origx = self.rectangle.width() / 2 + (radius-1) * math.cos(angle)
            origy = self.rectangle.height() / 2 + (radius-1) * math.sin(angle)
            finx = 6 * math.cos(angle) + origx
            finy = 6 * math.sin(angle) + origy
            self.setStyle(painter, Qt.black,Qt.black,3)
            painter.drawLine(QPoint(origx,origy), QPoint(finx,finy))
            angle = angle + (6*pi/180)


    def drawArrows(self, painter):
        radius = 130
        origx = self.rectangle.width() / 2
        origy = self.rectangle.height() / 2
        finx = radius * math.cos(self.angle) + origx
        finy = radius * math.sin(self.angle) + origy
        finMinutesx = radius/2 * math.cos(self.angleMinutes) + origx
        finMinutesy = radius/2 * math.sin(self.angleMinutes) + origy
        self.setStyle(painter, Qt.black,Qt.black,3)
        painter.drawLine(QPoint(origx,origy), QPoint(finx,finy))
        painter.drawLine(QPoint(origx,origy), QPoint(finMinutesx,finMinutesy))
        painter.drawEllipse(145,145, 10, 10)

    def accountantTime(self):
        if self.accountant < self.seconds:
            self.accountant += 1
            self.angle = self.angle + (6*pi/180)
        if self.accountant % 60 == 0:
            self.minutes += 1
            self.angleMinutes = self.angleMinutes + (6*pi/180)

    def resetPen(self, painter):
        pen = QPen(Qt.black, 1)
        brush = QBrush()
        painter.setPen(pen)
        painter.setBrush(brush)

    def setStyle(self, painter, fillColor, penColor, stroke):
        brush = QBrush()
        pen = QPen(penColor, stroke)
        brush.setColor(fillColor)
        brush.setStyle(Qt.SolidPattern)
        painter.setBrush(brush)
        painter.setPen(pen)
        painter.setRenderHint(QPainter.Antialiasing)

    def paintEvent(self, event):
        painter = QPainter(self)
        self.drawWhiteZones(painter)
        self.drawArrows(painter)
        self.drawCLockLines(painter)

    def updateG(self):
        self.update()

def quat2Yaw(qw, qx, qy, qz):
    '''
    Translates from Quaternion to Yaw. 

    @param qw,qx,qy,qz: Quaternion values

    @type qw,qx,qy,qz: float

    @return Yaw value translated from Quaternion

    '''
    rotateZa0=2.0*(qx*qy + qw*qz)
    rotateZa1=qw*qw + qx*qx - qy*qy - qz*qz
    rotateZ=0.0
    if(rotateZa0 != 0.0 and rotateZa1 != 0.0):
        rotateZ=atan2(rotateZa0,rotateZa1)
    return rotateZ

def quat2Pitch(qw, qx, qy, qz):
    '''
    Translates from Quaternion to Pitch. 

    @param qw,qx,qy,qz: Quaternion values

    @type qw,qx,qy,qz: float

    @return Pitch value translated from Quaternion

    '''

    rotateYa0=-2.0*(qx*qz - qw*qy)
    rotateY=0.0
    if(rotateYa0 >= 1.0):
        rotateY = pi/2.0
    elif(rotateYa0 <= -1.0):
        rotateY = -pi/2.0
    else:
        rotateY = asin(rotateYa0)

    return rotateY

def quat2Roll (qw, qx, qy, qz):
    '''
    Translates from Quaternion to Roll. 

    @param qw,qx,qy,qz: Quaternion values

    @type qw,qx,qy,qz: float

    @return Roll value translated from Quaternion

    '''
    rotateXa0=2.0*(qy*qz + qw*qx)
    rotateXa1=qw*qw - qx*qx - qy*qy + qz*qz
    rotateX=0.0

    if(rotateXa0 != 0.0 and rotateXa1 != 0.0):
        rotateX=atan2(rotateXa0, rotateXa1)
    return rotateX


def odometry2Pose3D(odom):
    '''
    Translates from ROS Odometry to JderobotTypes Pose3d. 

    @param odom: ROS Odometry to translate

    @type odom: Odometry

    @return a Pose3d translated from odom

    '''
    pose = Pose3d()
    ori = odom.pose.pose.orientation

    pose.x = odom.pose.pose.position.x
    pose.y = odom.pose.pose.position.y
    pose.z = odom.pose.pose.position.z
    #pose.h = odom.pose.pose.position.h
    pose.yaw = quat2Yaw(ori.w, ori.x, ori.y, ori.z)
    pose.pitch = quat2Pitch(ori.w, ori.x, ori.y, ori.z)
    pose.roll = quat2Roll(ori.w, ori.x, ori.y, ori.z)
    pose.q = [ori.w, ori.x, ori.y, ori.z]
    pose.timeStamp = odom.header.stamp.secs + (odom.header.stamp.nsecs *1e-9)

    return pose


class ListenerPose3d:
    '''
        ROS Pose3D Subscriber. Pose3D Client to Receive pose3d from ROS nodes.
    '''
    def __init__(self, topic):
        '''
        ListenerPose3d Constructor.

        @param topic: ROS topic to subscribe
        
        @type topic: String

        '''
        self.topic = topic
        self.data = Pose3d()
        self.sub = None
        self.lock = threading.Lock()
        self.start()
 
    def __callback (self, odom):
        '''
        Callback function to receive and save Pose3d. 

        @param odom: ROS Odometry received
        
        @type odom: Odometry

        '''
        pose = odometry2Pose3D(odom)

        self.lock.acquire()
        self.data = pose
        self.lock.release()
        
    def stop(self):
        '''
        Stops (Unregisters) the client.

        '''
        self.sub.unregister()

    def start (self):
        '''
        Starts (Subscribes) the client.

        '''
        self.sub = rospy.Subscriber(self.topic, Odometry, self.__callback)
        
    def getPose3d(self):
        '''
        Returns last Pose3d. 

        @return last JdeRobotTypes Pose3d saved

        '''
        self.lock.acquire()
        pose = self.data
        self.lock.release()
        
        return pose


if __name__ == "__main__":

    app = QApplication(sys.argv)
    cfg = config.load(sys.argv[1])
    ymlNode = cfg.getProperty('Referee')
    node = rospy.init_node(ymlNode["NodeName"], anonymous=True)

    print("Receiving " + "Referee.Pose3D" + " from ROS messages")
    topicP = cfg.getProperty("Referee.Pose3D"+".Topic")
    pose3d = ListenerPose3d(topicP)

    myGUI = MainWindow(pose3d)
    myGUI.show()
    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()
    sys.exit(app.exec_())
