# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '3d_reconstructrionui.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(733, 870)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(21, 17, 591, 411))
        self.layoutWidget.setObjectName("layoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.layoutWidget)
        #self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
	self.imageFiltered = QtWidgets.QLabel(self.layoutWidget)
        self.imageFiltered.setText("")
        self.imageFiltered.setObjectName("imageFiltered")
        self.gridLayout.addWidget(self.imageFiltered, 2, 1, 1, 1)
        self.image = QtWidgets.QLabel(self.layoutWidget)
        self.image.setObjectName("image")
        self.gridLayout.addWidget(self.image, 2, 0, 1, 1)
        self.label = QtWidgets.QLabel(self.layoutWidget)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 1, 0, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.layoutWidget)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 1, 1, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.pushButton = QtWidgets.QPushButton(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setItalic(True)
        font.setWeight(75)
        self.pushButton.setFont(font)
        self.pushButton.setObjectName("pushButton")
        self.horizontalLayout.addWidget(self.pushButton)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.layoutWidget1 = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget1.setGeometry(QtCore.QRect(50, 440, 520, 370))
        self.layoutWidget1.setObjectName("layoutWidget1")
        self.graphicLayout = QtWidgets.QVBoxLayout(self.layoutWidget1)
        #self.graphicLayout.setContentsMargins(0, 0, 0, 0)
        self.graphicLayout.setObjectName("graphicLayout")
        self.resetButton = QtWidgets.QPushButton(self.layoutWidget1)
        self.resetButton.setFont(font)
        self.resetButton.setObjectName("resetButton")
        self.graphicLayout.addWidget(self.resetButton)
        #self.graphicLayout.addWidget(self.graphicLayout, 2, 1, 1, 1)
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(570, 660, 160, 171))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.logoLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        #self.logoLayout.setContentsMargins(0, 0, 0, 0)
        self.logoLayout.setObjectName("logoLayout")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 733, 19))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.image.setText(_translate("MainWindow", "TextLabel"))
        self.label.setText(_translate("MainWindow", "Camera image"))
        self.label_2.setText(_translate("MainWindow", "Processed image"))
        self.pushButton.setText(_translate("MainWindow", "Play Code"))
        self.resetButton.setText(_translate("MainWindow", "Reset View"))

import resources_rc
