# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '3d_reconstructrionui_smaller.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!
#  Authors :
#       Carlos Awadallah Estévez<carlosawadallah@gmail.com>

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow, mapwidth, mapheight):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(mapwidth+(1200-mapwidth), mapheight+60)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.centralwidget.setObjectName("centralwidget")

        self.mainLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.mainLayout.setObjectName("mainLayout")

        self.verticalLayout_right = QtWidgets.QVBoxLayout()
        self.verticalLayout_right.setObjectName("verticalLayout_right")

        self.horizontalLayout_right = QtWidgets.QHBoxLayout()
        self.horizontalLayout_right.setObjectName("horizontalLayout_right")
        self.logoLayout = QtWidgets.QVBoxLayout()
        self.logoLayout.setContentsMargins(-80, -1, 0, 0)
        self.logoLayout.setObjectName("logoLayout")
        self.horizontalLayout_right.addLayout(self.logoLayout)

        self.tlLayout = QtWidgets.QVBoxLayout()
        self.tlLayout.setContentsMargins(60, -1, 60, -1)
        self.tlLayout.setObjectName("tlLayout")
        self.horizontalLayout_right.addLayout(self.tlLayout)

        # Debug Purposes
        # ---------------------------------------------------------------
        #self.rotationDial = QtWidgets.QDial(self.centralwidget)
        #self.rotationDial.setGeometry(QtCore.QRect(440, 220, 50, 64))
        #self.rotationDial.setMaximum(360)
        #self.rotationDial.setProperty("value", 180)
        #self.rotationDial.setObjectName("rotationDial")

        #self.probButton = QtWidgets.QPushButton(self.centralwidget)
        #font = QtGui.QFont()
        #font.setPointSize(15)
        #font.setBold(True)
        #font.setWeight(75)
        #self.probButton.setFont(font)
        #self.horizontalLayout_right.addWidget(self.rotationDial)
        #self.horizontalLayout_right.addWidget(self.probButton)

        #self.newGeneration = QtWidgets.QPushButton(self.centralwidget)
        #font = QtGui.QFont()
        #font.setPointSize(15)
        #font.setBold(True)
        #font.setWeight(75)
        #self.newGeneration.setFont(font)
        #self.horizontalLayout_right.addWidget(self.newGeneration)'''
        # ---------------------------------------------------------------
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")

        self.roboticsLogoLayout = QtWidgets.QVBoxLayout()
        self.roboticsLogoLayout.setContentsMargins(40, -1, 0, -1)
        self.roboticsLogoLayout.setObjectName("roboticsLogoLayout")
        
        self.horizontalLayout_right.addLayout(self.roboticsLogoLayout)

        self.map1Layout = QtWidgets.QVBoxLayout()
        self.map1Layout.setContentsMargins(1, -1, 0, -1)
        self.map1Layout.setObjectName("map1Layout")
        self.horizontalLayout_8.addLayout(self.map1Layout)
        self.verticalLayout_right.addLayout(self.horizontalLayout_right)
        self.verticalLayout_right.addLayout(self.horizontalLayout_8)

        self.mainLayout.addLayout(self.verticalLayout_right)

        self.verticalLayout_left = QtWidgets.QVBoxLayout()
        self.verticalLayout_left.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.verticalLayout_left.setObjectName("verticalLayout_left")
        self.mapLayout = QtWidgets.QVBoxLayout()
        self.mapLayout.setContentsMargins(1, -1, 0, -1)
        self.mapLayout.setObjectName("mapLayout")
        self.verticalLayout_left.addLayout(self.mapLayout)
        self.horizontalLayout_left = QtWidgets.QHBoxLayout()
        self.horizontalLayout_left.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)
        self.horizontalLayout_left.setSpacing(2)
        self.horizontalLayout_left.setObjectName("horizontalLayout_left")
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/images/play.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton.setIcon(icon)
        self.pushButton.setIconSize(QtCore.QSize(165,35))
        self.pushButton.setObjectName("pushButton")
        self.horizontalLayout_left.addWidget(self.pushButton)
        self.verticalLayout_left.addLayout(self.horizontalLayout_left)
        self.mainLayout.addLayout(self.verticalLayout_left)
        MainWindow.setCentralWidget(self.centralwidget)
        
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        
    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        #self.probButton.setText(_translate("MainWindow", "Recalculate"))
        #self.newGeneration.setText(_translate("MainWindow", "New Generation"))

import resources_rc
