# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '3d_reconstructrionui.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(860, 950)

        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(20, 20, 820, 440))
        self.layoutWidget.setObjectName("layoutWidget")

        self.verticalLayout = QtWidgets.QVBoxLayout(self.layoutWidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")

        # IMAGES
        self.label = QtWidgets.QLabel(self.layoutWidget)
        self.label.setObjectName("label")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setMinimumSize(QtCore.QSize(400,30))
        # self.label.setStyleSheet("background-color: blue")
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.layoutWidget)
        self.label_2.setObjectName("label_2")
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setMinimumSize(QtCore.QSize(400,30))
        # self.label_2.setStyleSheet("background-color: blue")
        self.gridLayout.addWidget(self.label_2, 0, 1, 1, 1)
        self.image = QtWidgets.QLabel(self.layoutWidget)
        self.image.setObjectName("image")
        self.image.setMinimumSize(QtCore.QSize(400,300))
        self.gridLayout.addWidget(self.image, 1, 0, 1, 1)
        self.imageFiltered = QtWidgets.QLabel(self.layoutWidget)
        self.imageFiltered.setObjectName("imageFiltered") 
        self.imageFiltered.setMinimumSize(QtCore.QSize(400,300))
        self.gridLayout.addWidget(self.imageFiltered, 1, 1, 1, 1)

        #BUTTONS
        self.pushButton = QtWidgets.QPushButton(self.layoutWidget)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("resources/play.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton.setIcon(icon)
        self.pushButton.setIconSize(QtCore.QSize(165,35))
        self.pushButton.setObjectName("pushButton")
        self.gridLayout.addWidget(self.pushButton, 2, 0, 1, 1)

        self.stopButton = QtWidgets.QPushButton(self.layoutWidget)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("resources/stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.stopButton.setIcon(icon)
        self.stopButton.setIconSize(QtCore.QSize(165,35))        
        self.stopButton.setObjectName("stopButton")
        self.gridLayout.addWidget(self.stopButton, 2, 1, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout)



        self.layoutWidgetMap = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidgetMap.setGeometry(QtCore.QRect(30, 480, 800, 380))
        self.layoutWidgetMap.setObjectName("layoutWidgetMap")

        self.verticalLayoutMap = QtWidgets.QVBoxLayout(self.layoutWidgetMap)
        self.verticalLayoutMap.setObjectName("verticalLayoutMap")

        self.mapLabel = QtWidgets.QLabel(self.layoutWidgetMap)
        self.mapLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.mapLabel.setFixedSize(QtCore.QSize(800,30))
        # self.mapLabel.setStyleSheet("background-color: orange")
        self.verticalLayoutMap.addWidget(self.mapLabel)

        
        pixmap = QtGui.QPixmap("/home/fran/Pictures/aa.jpg")
        self.image.setPixmap(pixmap)
        self.imageFiltered.setPixmap(pixmap)
        #self.image.show()
        # verticalSpacer = QtWidgets.QSpacerItem(20, 500, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)

        

        # self.verticalLayout.addLayout(self.horizontalLayout)
        # self.layoutWidget1 = QtWidgets.QWidget(self.centralwidget)
        # self.layoutWidget1.setGeometry(QtCore.QRect(140, 440, 331, 381))
        # self.layoutWidget1.setObjectName("layoutWidget1")
        
        #self.verticalLayout.addItem(verticalSpacer)

        # self.horizontalLayout = QtWidgets.QHBoxLayout()
        # self.horizontalLayout.setObjectName("horizontalLayout")

        # self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.layoutWidget1)
        # self.verticalLayout_2.setObjectName("verticalLayout_2")
        # self.tlLayout = QtWidgets.QVBoxLayout()
        # self.tlLayout.setObjectName("tlLayout")
        # self.verticalLayout_2.addLayout(self.tlLayout)
        # self.stopButton = QtWidgets.QPushButton(self.layoutWidget1)
        # font = QtGui.QFont()
        # font.setPointSize(15)
        # font.setBold(True)
        # font.setWeight(75)
        # self.stopButton.setFont(font)
        # icon = QtGui.QIcon()
        # icon.addPixmap(QtGui.QPixmap(":/images/stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        # self.stopButton.setIcon(icon)
        # self.stopButton.setObjectName("stopButton")
        # self.verticalLayout_2.addWidget(self.stopButton)
        # self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        # self.verticalLayoutWidget.setGeometry(QtCore.QRect(570, 660, 160, 171))
        # self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        # self.logoLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        # self.logoLayout.setObjectName("logoLayout")

        MainWindow.setCentralWidget(self.centralwidget)
        # self.menubar = QtWidgets.QMenuBar(MainWindow)
        # self.menubar.setGeometry(QtCore.QRect(0, 0, 733, 19))
        # self.menubar.setObjectName("menubar")
        # MainWindow.setMenuBar(self.menubar)
        # self.statusbar = QtWidgets.QStatusBar(MainWindow)
        # self.statusbar.setObjectName("statusbar")
        # MainWindow.setStatusBar(self.statusbar)
        self.retranslateUi(MainWindow)
        # QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        # self.image.setText(_translate("MainWindow", "TextLabel"))
        self.label.setText(_translate("MainWindow", "Camera image"))
        self.label_2.setText(_translate("MainWindow", "Processed image"))
        self.pushButton.setText(_translate("MainWindow", "Play Code"))
        self.stopButton.setText(_translate("MainWindow", "Stop Vehicle"))
        self.mapLabel.setText(_translate("MainWindow", "Map"))



import resources_rc
