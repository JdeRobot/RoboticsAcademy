# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '3d_reconstructrionui.ui'
#
# Created: Wed Jan 27 18:17:17 2016
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
        MainWindow.resize(1082, 867)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayoutWidget = QtGui.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(160, 480, 361, 301))
        self.verticalLayoutWidget.setObjectName(_fromUtf8("verticalLayoutWidget"))
        self.tlLayout = QtGui.QVBoxLayout(self.verticalLayoutWidget)
        self.tlLayout.setMargin(0)
        self.tlLayout.setObjectName(_fromUtf8("tlLayout"))
        self.widget = QtGui.QWidget(self.centralwidget)
        self.widget.setGeometry(QtCore.QRect(21, 17, 591, 411))
        self.widget.setObjectName(_fromUtf8("widget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.widget)
        self.verticalLayout.setMargin(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.imageLeftFiltered = QtGui.QLabel(self.widget)
        self.imageLeftFiltered.setObjectName(_fromUtf8("imageLeftFiltered"))
        self.gridLayout.addWidget(self.imageLeftFiltered, 2, 0, 1, 1)
        self.imageRightFiltered = QtGui.QLabel(self.widget)
        self.imageRightFiltered.setObjectName(_fromUtf8("imageRightFiltered"))
        self.gridLayout.addWidget(self.imageRightFiltered, 2, 1, 1, 1)
        self.imageLeft = QtGui.QLabel(self.widget)
        self.imageLeft.setObjectName(_fromUtf8("imageLeft"))
        self.gridLayout.addWidget(self.imageLeft, 1, 0, 1, 1)
        self.imageRight = QtGui.QLabel(self.widget)
        self.imageRight.setObjectName(_fromUtf8("imageRight"))
        self.gridLayout.addWidget(self.imageRight, 1, 1, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.stopButton = QtGui.QPushButton(self.widget)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8(":/images/stop.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.stopButton.setIcon(icon)
        self.stopButton.setObjectName(_fromUtf8("stopButton"))
        self.horizontalLayout.addWidget(self.stopButton)
        self.pushButton = QtGui.QPushButton(self.widget)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setItalic(True)
        font.setWeight(75)
        self.pushButton.setFont(font)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout.addWidget(self.pushButton)
        self.verticalLayout.addLayout(self.horizontalLayout)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1082, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.imageLeftFiltered.setText(_translate("MainWindow", "TextLabel", None))
        self.imageRightFiltered.setText(_translate("MainWindow", "TextLabel", None))
        self.imageLeft.setText(_translate("MainWindow", "TextLabel", None))
        self.imageRight.setText(_translate("MainWindow", "TextLabel", None))
        self.stopButton.setText(_translate("MainWindow", "Stop", None))
        self.pushButton.setText(_translate("MainWindow", "Run my algorithm", None))

import resources_rc
