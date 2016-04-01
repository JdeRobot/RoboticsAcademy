# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '3d_reconstructrionui.ui'
#
# Created: Fri Apr  1 19:05:08 2016
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(782, 560)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.imageLeft = QtGui.QLabel(self.centralwidget)
        self.imageLeft.setObjectName(_fromUtf8("imageLeft"))
        self.gridLayout.addWidget(self.imageLeft, 2, 0, 1, 1)
        self.imageRightFiltered = QtGui.QLabel(self.centralwidget)
        self.imageRightFiltered.setText(_fromUtf8(""))
        self.imageRightFiltered.setObjectName(_fromUtf8("imageRightFiltered"))
        self.gridLayout.addWidget(self.imageRightFiltered, 4, 1, 1, 1)
        self.imageLeftFiltered = QtGui.QLabel(self.centralwidget)
        self.imageLeftFiltered.setText(_fromUtf8(""))
        self.imageLeftFiltered.setObjectName(_fromUtf8("imageLeftFiltered"))
        self.gridLayout.addWidget(self.imageLeftFiltered, 4, 0, 1, 1)
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setObjectName(_fromUtf8("label"))
        self.gridLayout.addWidget(self.label, 1, 0, 1, 1)
        self.imageRight = QtGui.QLabel(self.centralwidget)
        self.imageRight.setObjectName(_fromUtf8("imageRight"))
        self.gridLayout.addWidget(self.imageRight, 2, 1, 1, 1)
        self.label_2 = QtGui.QLabel(self.centralwidget)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.gridLayout.addWidget(self.label_2, 3, 0, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout)
        self.horizontalLayout_2.addLayout(self.verticalLayout)
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.tlLayout = QtGui.QVBoxLayout()
        self.tlLayout.setObjectName(_fromUtf8("tlLayout"))
        self.verticalLayout_2.addLayout(self.tlLayout)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.pushButton = QtGui.QPushButton(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setItalic(True)
        font.setWeight(75)
        self.pushButton.setFont(font)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout.addWidget(self.pushButton)
        self.verticalLayout_2.addLayout(self.horizontalLayout)
        self.stopButton = QtGui.QPushButton(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.stopButton.setFont(font)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8(":/images/stop.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.stopButton.setIcon(icon)
        self.stopButton.setObjectName(_fromUtf8("stopButton"))
        self.verticalLayout_2.addWidget(self.stopButton)
        self.horizontalLayout_2.addLayout(self.verticalLayout_2)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 782, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.imageLeft.setText(_translate("MainWindow", "TextLabel", None))
        self.label.setText(_translate("MainWindow", "Input", None))
        self.imageRight.setText(_translate("MainWindow", "TextLabel", None))
        self.label_2.setText(_translate("MainWindow", "Processed images", None))
        self.pushButton.setText(_translate("MainWindow", "Run my algorithm", None))
        self.stopButton.setText(_translate("MainWindow", "Stop", None))

import resources_rc
