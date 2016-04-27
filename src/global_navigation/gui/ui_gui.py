# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'teleTaxigui.ui'
#
# Created: Fri Apr  8 16:03:12 2016
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
        MainWindow.resize(837, 534)
        MainWindow.setMinimumSize(QtCore.QSize(660, 370))
        MainWindow.setMaximumSize(QtCore.QSize(1200, 1200))
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayoutWidget = QtGui.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(20, 40, 401, 391))
        self.verticalLayoutWidget.setObjectName(_fromUtf8("verticalLayoutWidget"))
        self.mapLayout = QtGui.QVBoxLayout(self.verticalLayoutWidget)
        self.mapLayout.setMargin(0)
        self.mapLayout.setObjectName(_fromUtf8("mapLayout"))
        self.verticalLayoutWidget_2 = QtGui.QWidget(self.centralwidget)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(440, 40, 371, 391))
        self.verticalLayoutWidget_2.setObjectName(_fromUtf8("verticalLayoutWidget_2"))
        self.tlLayout = QtGui.QVBoxLayout(self.verticalLayoutWidget_2)
        self.tlLayout.setMargin(0)
        self.tlLayout.setObjectName(_fromUtf8("tlLayout"))
        self.getPathButton = QtGui.QPushButton(self.centralwidget)
        self.getPathButton.setGeometry(QtCore.QRect(50, 470, 131, 51))
        self.getPathButton.setObjectName(_fromUtf8("getPathButton"))
        self.playButton = QtGui.QPushButton(self.centralwidget)
        self.playButton.setGeometry(QtCore.QRect(270, 470, 101, 51))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8(":/images/play.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.playButton.setIcon(icon)
        self.playButton.setObjectName(_fromUtf8("playButton"))
        self.stopButton = QtGui.QPushButton(self.centralwidget)
        self.stopButton.setGeometry(QtCore.QRect(430, 470, 111, 51))
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(_fromUtf8(":/images/stop.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.stopButton.setIcon(icon1)
        self.stopButton.setObjectName(_fromUtf8("stopButton"))
        self.destinyLabel = QtGui.QLabel(self.centralwidget)
        self.destinyLabel.setGeometry(QtCore.QRect(30, 430, 61, 21))
        self.destinyLabel.setObjectName(_fromUtf8("destinyLabel"))
        self.XLabel = QtGui.QLabel(self.centralwidget)
        self.XLabel.setGeometry(QtCore.QRect(100, 440, 21, 21))
        self.XLabel.setObjectName(_fromUtf8("XLabel"))
        self.YLabel = QtGui.QLabel(self.centralwidget)
        self.YLabel.setGeometry(QtCore.QRect(220, 440, 21, 21))
        self.YLabel.setObjectName(_fromUtf8("YLabel"))
        self.positionLabel = QtGui.QLabel(self.centralwidget)
        self.positionLabel.setGeometry(QtCore.QRect(440, 430, 61, 20))
        self.positionLabel.setObjectName(_fromUtf8("positionLabel"))
        self.VLabel = QtGui.QLabel(self.centralwidget)
        self.VLabel.setGeometry(QtCore.QRect(530, 440, 21, 21))
        self.VLabel.setObjectName(_fromUtf8("VLabel"))
        self.XValue = QtGui.QLabel(self.centralwidget)
        self.XValue.setGeometry(QtCore.QRect(130, 440, 41, 21))
        self.XValue.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.XValue.setObjectName(_fromUtf8("XValue"))
        self.VValue = QtGui.QLabel(self.centralwidget)
        self.VValue.setGeometry(QtCore.QRect(560, 440, 41, 21))
        self.VValue.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.VValue.setObjectName(_fromUtf8("VValue"))
        self.YValue = QtGui.QLabel(self.centralwidget)
        self.YValue.setGeometry(QtCore.QRect(260, 440, 41, 21))
        self.YValue.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.YValue.setObjectName(_fromUtf8("YValue"))
        self.WValue = QtGui.QLabel(self.centralwidget)
        self.WValue.setGeometry(QtCore.QRect(700, 440, 41, 21))
        self.WValue.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.WValue.setObjectName(_fromUtf8("WValue"))
        self.WLabel = QtGui.QLabel(self.centralwidget)
        self.WLabel.setGeometry(QtCore.QRect(660, 440, 21, 21))
        self.WLabel.setObjectName(_fromUtf8("WLabel"))
        self.mapLabel = QtGui.QLabel(self.centralwidget)
        self.mapLabel.setGeometry(QtCore.QRect(20, 10, 251, 17))
        self.mapLabel.setObjectName(_fromUtf8("mapLabel"))
        self.mapLabel_2 = QtGui.QLabel(self.centralwidget)
        self.mapLabel_2.setGeometry(QtCore.QRect(440, 10, 121, 17))
        self.mapLabel_2.setObjectName(_fromUtf8("mapLabel_2"))
        self.colorFilter = QtGui.QCheckBox(self.centralwidget)
        self.colorFilter.setGeometry(QtCore.QRect(590, 480, 111, 22))
        self.colorFilter.setObjectName(_fromUtf8("colorFilter"))
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "Introrob py", None))
        self.getPathButton.setText(_translate("MainWindow", "Generate Path", None))
        self.playButton.setText(_translate("MainWindow", "GO!", None))
        self.stopButton.setText(_translate("MainWindow", "Stop", None))
        self.destinyLabel.setText(_translate("MainWindow", "Destiny:", None))
        self.XLabel.setText(_translate("MainWindow", "X:", None))
        self.YLabel.setText(_translate("MainWindow", "Y:", None))
        self.positionLabel.setText(_translate("MainWindow", "Position:", None))
        self.VLabel.setText(_translate("MainWindow", "V:", None))
        self.XValue.setText(_translate("MainWindow", "0", None))
        self.VValue.setText(_translate("MainWindow", "0", None))
        self.YValue.setText(_translate("MainWindow", "0", None))
        self.WValue.setText(_translate("MainWindow", "0", None))
        self.WLabel.setText(_translate("MainWindow", "W:", None))
        self.mapLabel.setText(_translate("MainWindow", "MAP: double click for selecting your destiny.", None))
        self.mapLabel_2.setText(_translate("MainWindow", "Manual controler:", None))
        self.colorFilter.setText(_translate("MainWindow", "Color Filter", None))

from resources import resources_rc
