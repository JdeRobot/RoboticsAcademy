# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui_gui.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

import resources_rc
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSignal, Qt, QSize
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget, QLabel, QHBoxLayout, QVBoxLayout, QGroupBox, QCheckBox
from PyQt5.QtGui import QImage, QPixmap

from gui.communicator import Communicator
from gui.logoWidget import LogoWidget


class Ui_MainWindow(object):
    IMAGE_COLS_MAX = 640
    IMAGE_ROWS_MAX = 360
    LINX = 0.3
    LINY = 0.3
    LINZ = 0.8
    ANGZ = 1.0
    ANGY = 0.0
    ANGX = 0.0

    def setupUI(self, MainWindow):
        ## All GUI components of the main window are built here

        self.setWindowTitle("Color filter")

        # VerticalLayout with 2 rows
        vlayout = QVBoxLayout(MainWindow)
        vlayout.setSpacing(0)
        
        vlayout.setContentsMargins(0,0,0,0)
        # Row 1
        groupbox_1 = QGroupBox()
        groupbox_1.setContentsMargins(0,0,0,0)
        hlayout = QHBoxLayout()
        hlayout.setContentsMargins(0,0,0,0)
        
        hlayout.setAlignment(Qt.AlignCenter)

        # Row 1 Col 1
        groupbox_1_1 = QGroupBox()
        vlayout_inner = QVBoxLayout()
        vlayout_inner.setContentsMargins(0,0,0,0)
        vlayout_inner.setAlignment(Qt.AlignCenter)
        text1 = "Live Video Stream"
        window_1_label = QLabel()
        window_1_label.setAlignment(Qt.AlignCenter)
        window_1_label.setText(text1)
        vlayout_inner.addWidget(window_1_label)
        self.imgLabelColor = QLabel()
        self.imgLabelColor.setFixedSize(640, 360)
        self.imgLabelColor.setAlignment(Qt.AlignCenter)
        vlayout_inner.addWidget(self.imgLabelColor)
        groupbox_1_1.setLayout(vlayout_inner)
        hlayout.addWidget(groupbox_1_1)

        # Row 1 Col 2

        groupbox_1_2 = QGroupBox()
        vlayout_inner = QVBoxLayout()
        vlayout_inner.setContentsMargins(0,0,0,0)
        vlayout_inner.setAlignment(Qt.AlignCenter)
        text2 = "Output after thresholding"
        window_2_label = QLabel()
        window_2_label.setAlignment(Qt.AlignCenter)
        window_2_label.setText(text2)
        vlayout_inner.addWidget(window_2_label)
        self.imgLabelBlackWhite = QLabel()
        self.imgLabelBlackWhite.setAlignment(Qt.AlignCenter)
        self.imgLabelBlackWhite.setFixedSize(640, 360)
        vlayout_inner.addWidget(self.imgLabelBlackWhite)
        groupbox_1_2.setLayout(vlayout_inner)
        hlayout.addWidget(groupbox_1_2)
        groupbox_1.setLayout(hlayout)
        vlayout.addWidget(groupbox_1)

        
        # ROW 2

        groupbox_2 = QGroupBox()
        groupbox_2.setContentsMargins(0,0,0,0)
        h_3layout = QHBoxLayout()
        h_3layout.setContentsMargins(0,0,0,0)

        # Row 2 Col 1

        groupbox_2_1 = QGroupBox()
        
        h2layout = QVBoxLayout()
        h2layout.setAlignment(Qt.AlignCenter)
        text3 = 'Final Processed Image'
        window_3_label = QLabel()
        window_3_label.setAlignment(Qt.AlignCenter)
        window_3_label.setText(text3)
        h2layout.addWidget(window_3_label)
        self.detectedImage = QLabel()
        self.detectedImage.setAlignment(Qt.AlignCenter)
        self.detectedImage.setFixedSize(640, 360)
        h2layout.addWidget(self.detectedImage)
        groupbox_2_1.setLayout(h2layout)
        h_3layout.addWidget(groupbox_2_1, 75)

        # Row 2 Col 2
        groupbox_2_2 = QGroupBox()
        self.v3_layout = QVBoxLayout()
        self.v3_layout.setAlignment(Qt.AlignCenter)
        self.v3_layout.addSpacing(30)
        self.v3_layout.setAlignment(Qt.AlignCenter)
        self.centralwidget = QtWidgets.QWidget()
        self.playButton = QtWidgets.QPushButton(self.centralwidget)
        self.playButton.setFixedSize(QSize(200, 100))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/images/play.png"),
                       QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.playButton.setObjectName("playButton")
        self.v3_layout.addWidget(self.playButton, 60)
        self.logo = LogoWidget(self, 200, 100)
        self.logo.setVisible(True)
        self.v3_layout.addWidget(self.logo, 40)
        groupbox_2_2.setLayout(self.v3_layout)
        h_3layout.addWidget(groupbox_2_2, 25)
        groupbox_2.setLayout(h_3layout)
        vlayout.addWidget(groupbox_2)
       
        icon = QtGui.QIcon()

        self.playButton.setIcon(icon)

        self.retranslateUi(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        self.playButton.setText(_translate("MainWindow", "Play Code"))

    def getCamera(self):
        return self.camera

    def setCamera(self, camera):
        self.camera = camera

    def setAlgorithm(self, algorithm):
        self.algorithm = algorithm

    def getAlgorithm(self):
        return self.algorithm

    def playClicked(self):
        if self.playButton.isChecked():
            icon = QtGui.QIcon()
            self.playButton.setText("Stop Code")
            self.playButton.setStyleSheet("background-color: #ec7063")
            icon.addPixmap(QtGui.QPixmap(":/images/stop.png"),
                           QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.playButton.setIcon(icon)
            self.algorithm.play()
        else:
            icon = QtGui.QIcon()
            icon.addPixmap(QtGui.QPixmap(":/images/play.png"),
                           QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.playButton.setIcon(icon)
            self.playButton.setText("Play Code")
            self.playButton.setStyleSheet("background-color: #7dcea0")
            self.algorithm.stop()

    def closeEvent(self, event):
        self.algorithm.kill()
        self.camera.client.stop()

        event.accept()

    def setColorImage(self):
        img = self.camera.getImage()

        if img is not None:
            image = QImage(
                img.data, img.shape[1], img.shape[0], img.shape[1] * img.shape[2], QImage.Format_RGB888)

            self.imgLabelColor.setPixmap(QPixmap.fromImage(image))

    def setThresholdImage(self):
        img = self.getCamera().getThresholdImage()
        if img is not None:
            image = QImage(
                img.data, img.shape[1], img.shape[0], img.shape[1] * img.shape[2], QImage.Format_RGB888)
            self.imgLabelBlackWhite.setPixmap(QPixmap.fromImage(image))

    def setDetectImage(self):
        img = self.getCamera().getDetectImage()
        if img is not None:
            image = QImage(
                img.data, img.shape[1], img.shape[0], img.shape[1] * img.shape[2], QImage.Format_RGB888)
            image1 = image.scaledToHeight(360)
            self.detectedImage.setPixmap(QPixmap.fromImage(image1))

    def updateImage(self):
        
        self.setThresholdImage()
        self.setDetectImage()

    def updateCamImage(self):
        self.setColorImage()
        

