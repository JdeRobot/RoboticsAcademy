# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui_gui.ui'
#
# Created: Sun Oct 11 12:42:26 2015
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
        MainWindow.resize(660, 370)
        MainWindow.setMinimumSize(QtCore.QSize(660, 370))
        MainWindow.setMaximumSize(QtCore.QSize(660, 370))
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.takeoffButton = QtGui.QPushButton(self.centralwidget)
        self.takeoffButton.setGeometry(QtCore.QRect(470, 30, 161, 41))
        self.takeoffButton.setObjectName(_fromUtf8("takeoffButton"))
        self.altdSlider = QtGui.QSlider(self.centralwidget)
        self.altdSlider.setGeometry(QtCore.QRect(400, 30, 19, 311))
        self.altdSlider.setMaximum(100)
        self.altdSlider.setProperty("value", 49)
        self.altdSlider.setOrientation(QtCore.Qt.Vertical)
        self.altdSlider.setObjectName(_fromUtf8("altdSlider"))
        self.playButton = QtGui.QPushButton(self.centralwidget)
        self.playButton.setGeometry(QtCore.QRect(470, 80, 71, 51))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8(":/images/play.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.playButton.setIcon(icon)
        self.playButton.setObjectName(_fromUtf8("playButton"))
        self.stopButton = QtGui.QPushButton(self.centralwidget)
        self.stopButton.setGeometry(QtCore.QRect(560, 80, 71, 51))
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(_fromUtf8(":/images/stop.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.stopButton.setIcon(icon1)
        self.stopButton.setObjectName(_fromUtf8("stopButton"))
        self.windowsLabel = QtGui.QLabel(self.centralwidget)
        self.windowsLabel.setGeometry(QtCore.QRect(540, 190, 71, 21))
        self.windowsLabel.setObjectName(_fromUtf8("windowsLabel"))
        self.cameraCheck = QtGui.QCheckBox(self.centralwidget)
        self.cameraCheck.setGeometry(QtCore.QRect(540, 220, 94, 26))
        self.cameraCheck.setObjectName(_fromUtf8("cameraCheck"))
        self.sensorsCheck = QtGui.QCheckBox(self.centralwidget)
        self.sensorsCheck.setGeometry(QtCore.QRect(540, 250, 94, 26))
        self.sensorsCheck.setObjectName(_fromUtf8("sensorsCheck"))
        self.colorFilterCheck = QtGui.QCheckBox(self.centralwidget)
        self.colorFilterCheck.setGeometry(QtCore.QRect(540,280,94,26))
        self.colorFilterCheck.setObjectName(_fromUtf8("colorFilterCheck"))
        self.altdLabel = QtGui.QLabel(self.centralwidget)
        self.altdLabel.setGeometry(QtCore.QRect(390, 340, 51, 21))
        self.altdLabel.setObjectName(_fromUtf8("altdLabel"))
        self.verticalLayoutWidget = QtGui.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(20, 30, 361, 301))
        self.verticalLayoutWidget.setObjectName(_fromUtf8("verticalLayoutWidget"))
        self.tlLayout = QtGui.QVBoxLayout(self.verticalLayoutWidget)
        self.tlLayout.setMargin(0)
        self.tlLayout.setObjectName(_fromUtf8("tlLayout"))
        self.rotationDial = QtGui.QDial(self.centralwidget)
        self.rotationDial.setGeometry(QtCore.QRect(440, 220, 50, 64))
        self.rotationDial.setMaximum(100)
        self.rotationDial.setProperty("value", 49)
        self.rotationDial.setObjectName(_fromUtf8("rotationDial"))
        self.rotationLabel = QtGui.QLabel(self.centralwidget)
        self.rotationLabel.setGeometry(QtCore.QRect(440, 280, 65, 21))
        self.rotationLabel.setObjectName(_fromUtf8("rotationLabel"))
        self.XLabel = QtGui.QLabel(self.centralwidget)
        self.XLabel.setGeometry(QtCore.QRect(20, 340, 21, 21))
        self.XLabel.setObjectName(_fromUtf8("XLabel"))
        self.YLabel = QtGui.QLabel(self.centralwidget)
        self.YLabel.setGeometry(QtCore.QRect(130, 340, 21, 21))
        self.YLabel.setObjectName(_fromUtf8("YLabel"))
        self.XValue = QtGui.QLabel(self.centralwidget)
        self.XValue.setGeometry(QtCore.QRect(40, 340, 41, 21))
        self.XValue.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.XValue.setObjectName(_fromUtf8("XValue"))
        self.YValue = QtGui.QLabel(self.centralwidget)
        self.YValue.setGeometry(QtCore.QRect(150, 340, 41, 21))
        self.YValue.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.YValue.setObjectName(_fromUtf8("YValue"))
        self.altdValue = QtGui.QLabel(self.centralwidget)
        self.altdValue.setGeometry(QtCore.QRect(390, 10, 41, 21))
        self.altdValue.setAlignment(QtCore.Qt.AlignCenter)
        self.altdValue.setObjectName(_fromUtf8("altdValue"))
        self.rotValue = QtGui.QLabel(self.centralwidget)
        self.rotValue.setGeometry(QtCore.QRect(445, 200, 41, 21))
        self.rotValue.setAlignment(QtCore.Qt.AlignCenter)
        self.rotValue.setObjectName(_fromUtf8("rotValue"))
        self.resetButton = QtGui.QPushButton(self.centralwidget)
        self.resetButton.setGeometry(QtCore.QRect(470, 140, 161, 41))
        self.resetButton.setObjectName(_fromUtf8("resetButton"))
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "Introrob py", None))
        self.takeoffButton.setText(_translate("MainWindow", "Take off", None))
        self.playButton.setText(_translate("MainWindow", "Play", None))
        self.stopButton.setText(_translate("MainWindow", "Stop", None))
        self.windowsLabel.setText(_translate("MainWindow", "Windows:", None))
        self.cameraCheck.setText(_translate("MainWindow", "Camera", None))
        self.sensorsCheck.setText(_translate("MainWindow", "Sensors", None))
        self.colorFilterCheck.setText(_translate("MainWindow", "Color filter", None))
        self.altdLabel.setText(_translate("MainWindow", "Altitude", None))
        self.rotationLabel.setText(_translate("MainWindow", "Rotation", None))
        self.XLabel.setText(_translate("MainWindow", "X:", None))
        self.YLabel.setText(_translate("MainWindow", "Y:", None))
        self.XValue.setText(_translate("MainWindow", "0", None))
        self.YValue.setText(_translate("MainWindow", "0", None))
        self.altdValue.setText(_translate("MainWindow", "0", None))
        self.rotValue.setText(_translate("MainWindow", "0", None))
        self.resetButton.setText(_translate("MainWindow", "Reset", None))

from resources import resources_rc

