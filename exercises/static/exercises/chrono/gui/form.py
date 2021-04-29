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
        MainWindow.resize(860, 1000)

        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(10, 10, 820, 440))
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
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.layoutWidget)
        self.label_2.setObjectName("label_2")
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setMinimumSize(QtCore.QSize(400,30))
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

        #MAP
        self.layoutWidgetMap = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidgetMap.setGeometry(QtCore.QRect(10, 450, 800, 380))
        self.layoutWidgetMap.setObjectName("layoutWidgetMap")

        self.verticalLayoutMap = QtWidgets.QVBoxLayout(self.layoutWidgetMap)
        self.verticalLayoutMap.setObjectName("verticalLayoutMap")

        self.mapLabel = QtWidgets.QLabel(self.layoutWidgetMap)
        self.mapLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.mapLabel.setFixedSize(QtCore.QSize(800,30))
        self.verticalLayoutMap.addWidget(self.mapLabel)

        pixmap = QtGui.QPixmap("/home/fran/Pictures/aa.jpg")
        self.image.setPixmap(pixmap)
        self.imageFiltered.setPixmap(pixmap)

        #CHRONO
        self.layoutWidgetChrono = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidgetChrono.setGeometry(QtCore.QRect(50, 820, 750, 250))
        self.layoutWidgetChrono.setObjectName("layoutWidgetChrono")

        self.verticalLayoutChrono = QtWidgets.QVBoxLayout(self.layoutWidgetChrono)
        self.verticalLayoutChrono.setObjectName("verticalLayoutChrono")
        self.gridLayoutChrono = QtWidgets.QGridLayout()
        self.gridLayoutChrono.setObjectName("gridLayoutChrono")

        self.label_chrono_1 = QtWidgets.QLabel(self.layoutWidgetChrono)
        self.label_chrono_1.setObjectName("label_chrono_1")
        self.label_chrono_1.setStyleSheet("font-weight:800; color: blue")
        self.label_chrono_1.setAlignment(QtCore.Qt.AlignCenter)
        self.label_chrono_1.setMinimumSize(QtCore.QSize(50,10))
        self.gridLayoutChrono.addWidget(self.label_chrono_1, 0, 0, 1, 1)
        self.chrono_1 = QtWidgets.QLabel(self.layoutWidgetChrono)
        self.chrono_1.setObjectName("Chrono 1")
        self.chrono_1.setStyleSheet("font-weight:500; color: blue")
        self.chrono_1.setAlignment(QtCore.Qt.AlignCenter)
        self.chrono_1.setMinimumSize(QtCore.QSize(50,50))
        self.gridLayoutChrono.addWidget(self.chrono_1, 1, 0, 1, 1)
        self.label_chrono_2 = QtWidgets.QLabel(self.layoutWidgetChrono)
        self.label_chrono_2.setObjectName("label_chrono_2")
        self.label_chrono_2.setStyleSheet("font-weight:800; color: blue")
        self.label_chrono_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_chrono_2.setMinimumSize(QtCore.QSize(50,10))
        self.gridLayoutChrono.addWidget(self.label_chrono_2, 0, 1, 1, 1)
        self.chrono_2 = QtWidgets.QLabel(self.layoutWidgetChrono)
        self.chrono_2.setObjectName("Chrono 2")
        self.chrono_2.setStyleSheet("font-weight:500; color: blue")
        self.chrono_2.setAlignment(QtCore.Qt.AlignCenter)
        self.chrono_2.setMinimumSize(QtCore.QSize(50,50))
        self.gridLayoutChrono.addWidget(self.chrono_2, 1, 1, 1, 1)

        self.verticalLayoutChrono.addLayout(self.gridLayoutChrono)

        MainWindow.setCentralWidget(self.centralwidget)
        self.retranslateUi(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        # self.image.setText(_translate("MainWindow", "TextLabel"))
        self.label.setText(_translate("MainWindow", "Camera image"))
        self.label_2.setText(_translate("MainWindow", "Processed image"))
        self.pushButton.setText(_translate("MainWindow", "Play Code"))
        self.stopButton.setText(_translate("MainWindow", "Stop Vehicle"))
        self.mapLabel.setText(_translate("MainWindow", "Map"))
        self.label_chrono_1.setText(_translate("MainWindow", "Chronometer"))
        self.label_chrono_2.setText(_translate("MainWindow", "Best lap"))



import resources_rc
