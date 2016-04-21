# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '3d_reconstructrionui_smaller.ui'
#
# Created: Thu Apr 21 18:26:23 2016
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
        MainWindow.resize(936, 469)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout_3 = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.tlLayout = QtGui.QVBoxLayout()
        self.tlLayout.setContentsMargins(0, -1, 1, -1)
        self.tlLayout.setObjectName(_fromUtf8("tlLayout"))
        self.horizontalLayout_2.addLayout(self.tlLayout)
        self.mapLayout = QtGui.QVBoxLayout()
        self.mapLayout.setContentsMargins(1, -1, 0, -1)
        self.mapLayout.setObjectName(_fromUtf8("mapLayout"))
        self.horizontalLayout_2.addLayout(self.mapLayout)
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)
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
        self.horizontalLayout_3.addLayout(self.verticalLayout_2)
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setSizeConstraint(QtGui.QLayout.SetDefaultConstraint)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.imageLeft = QtGui.QLabel(self.centralwidget)
        self.imageLeft.setObjectName(_fromUtf8("imageLeft"))
        self.gridLayout.addWidget(self.imageLeft, 2, 0, 1, 1)
        self.imageLeftFiltered = QtGui.QLabel(self.centralwidget)
        self.imageLeftFiltered.setText(_fromUtf8(""))
        self.imageLeftFiltered.setObjectName(_fromUtf8("imageLeftFiltered"))
        self.gridLayout.addWidget(self.imageLeftFiltered, 4, 0, 1, 1)
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setObjectName(_fromUtf8("label"))
        self.gridLayout.addWidget(self.label, 1, 0, 1, 1)
        self.imageRightFiltered = QtGui.QLabel(self.centralwidget)
        self.imageRightFiltered.setText(_fromUtf8(""))
        self.imageRightFiltered.setObjectName(_fromUtf8("imageRightFiltered"))
        self.gridLayout.addWidget(self.imageRightFiltered, 4, 1, 1, 1)
        self.imageRight = QtGui.QLabel(self.centralwidget)
        self.imageRight.setObjectName(_fromUtf8("imageRight"))
        self.gridLayout.addWidget(self.imageRight, 2, 1, 1, 1)
        self.label_2 = QtGui.QLabel(self.centralwidget)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.gridLayout.addWidget(self.label_2, 3, 0, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setSpacing(2)
        self.horizontalLayout.setSizeConstraint(QtGui.QLayout.SetMinimumSize)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.pushButton = QtGui.QPushButton(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton.sizePolicy().hasHeightForWidth())
        self.pushButton.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setItalic(True)
        font.setWeight(75)
        self.pushButton.setFont(font)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout.addWidget(self.pushButton)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_3.addLayout(self.verticalLayout)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 936, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.stopButton.setText(_translate("MainWindow", "Stop", None))
        self.imageLeft.setText(_translate("MainWindow", "TextLabel", None))
        self.label.setText(_translate("MainWindow", "Input", None))
        self.imageRight.setText(_translate("MainWindow", "TextLabel", None))
        self.label_2.setText(_translate("MainWindow", "Processed images", None))
        self.pushButton.setText(_translate("MainWindow", "Run my algorithm", None))

import resources_rc
