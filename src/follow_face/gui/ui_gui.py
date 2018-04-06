# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui_gui.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(660, 370)
        MainWindow.setMinimumSize(QtCore.QSize(660, 370))
        MainWindow.setMaximumSize(QtCore.QSize(660, 370))
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.playButton = QtWidgets.QPushButton(self.centralwidget)
        self.playButton.setGeometry(QtCore.QRect(400, 50, 111, 75))
        self.icon = QtGui.QIcon()
        self.icon.addPixmap(QtGui.QPixmap(":/images/play.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.playstopButton.setIcon(icon)
        self.playstopButton.setObjectName("playstopButton")
        self.playstopButton.setCheckable(True) 
        self.playstopButton.setChecked(True) 
        self.playstopButton.toggle()
        #self.stopButton = QtWidgets.QPushButton(self.centralwidget)
        #self.stopButton.setGeometry(QtCore.QRect(520, 50, 111, 75))
        self.icon1 = QtGui.QIcon()
        self.icon1.addPixmap(QtGui.QPixmap(":/images/stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        #self.stopButton.setIcon(icon1)
        #self.stopButton.setObjectName("stopButton")
        self.windowsLabel = QtWidgets.QLabel(self.centralwidget)
        self.windowsLabel.setGeometry(QtCore.QRect(490, 205, 71, 21))
        self.windowsLabel.setObjectName("windowsLabel")

        self.segmentCheck = QtWidgets.QCheckBox(self.centralwidget)
        self.segmentCheck.setGeometry(QtCore.QRect(460, 235, 150, 26))
        self.segmentCheck.setObjectName("segmentCheck")
        self.S1Label = QtWidgets.QLabel(self.centralwidget)
        self.S1Label.setGeometry(QtCore.QRect(460, 275, 150, 26))
        self.S1Label.setObjectName("S1Label")
        self.S2Label = QtWidgets.QLabel(self.centralwidget)
        self.S2Label.setGeometry(QtCore.QRect(460, 295, 150, 26))
        self.S2Label.setObjectName("S2Label")

        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(20, 30, 361, 301))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.tlLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.tlLayout.setObjectName("tlLayout")
        self.XLabel = QtWidgets.QLabel(self.centralwidget)
        self.XLabel.setGeometry(QtCore.QRect(115, 340, 28, 21))
        self.XLabel.setObjectName("XLabel")
        self.YLabel = QtWidgets.QLabel(self.centralwidget)
        self.YLabel.setGeometry(QtCore.QRect(225, 340, 25, 21))
        self.YLabel.setObjectName("YLabel")
        self.XValue = QtWidgets.QLabel(self.centralwidget)
        self.XValue.setGeometry(QtCore.QRect(135, 340, 41, 21))
        self.XValue.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.XValue.setObjectName("XValue")
        self.YValue = QtWidgets.QLabel(self.centralwidget)
        self.YValue.setGeometry(QtCore.QRect(245, 340, 41, 21))
        self.YValue.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.YValue.setObjectName("YValue")
        self.resetButton = QtWidgets.QPushButton(self.centralwidget)
        self.resetButton.setGeometry(QtCore.QRect(435, 140, 161, 41))
        self.resetButton.setObjectName("resetButton")
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(580, 290, 71, 71))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.logoLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.logoLayout.setSpacing(0)
        self.logoLayout.setObjectName("logoLayout")
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
    
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Follow Face"))
        self.playstopButton.setText(_translate("MainWindow", "Play"))
        #self.stopButton.setText(_translate("MainWindow", "Stop"))
        self.windowsLabel.setText(_translate("MainWindow", "Windows:"))

        self.segmentCheck.setText(_translate("MainWindow", "Segmented Image"))
        self.S1Label.setText(_translate("MainWindow", "Press Play Button to"))        
        self.S2Label.setText(_translate("MainWindow", "see camera's Image"))

        self.XLabel.setText(_translate("MainWindow", "Pan:"))
        self.YLabel.setText(_translate("MainWindow", "Tilt:"))
        self.XValue.setText(_translate("MainWindow", "0"))
        self.YValue.setText(_translate("MainWindow", "0"))
        self.resetButton.setText(_translate("MainWindow", "Reset"))

import resources_rc
