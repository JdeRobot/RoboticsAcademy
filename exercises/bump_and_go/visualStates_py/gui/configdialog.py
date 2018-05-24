'''
   Copyright (C) 1997-2017 JDERobot Developers Team

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Library General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, see <http://www.gnu.org/licenses/>.

   Authors : Okan Asik (asik.okan@gmail.com)

  '''
import sys
from PyQt5.QtWidgets import QDialog, QApplication, QGroupBox, QComboBox,\
    QVBoxLayout, QFormLayout, QGridLayout, QHBoxLayout, QWidget
from PyQt5.QtGui import QFontDatabase
from gui.jderobotcommconfigdialog import JdeRobotCommConfigDialog
from gui.rosconfigdialog import RosConfigDialog
from gui.config import ROS, JDEROBOTCOMM, RosConfig, JdeRobotConfig
from PyQt5.QtCore import pyqtSignal

class ConfigDialog(QDialog):
    configChanged = pyqtSignal()
    def __init__(self, title, config):
        QDialog.__init__(self)
        if config is not None:
            self.type = config.type
        else:
            self.type = JDEROBOTCOMM

        self.setWindowTitle(title)
        commSelectionBox = QGroupBox('Select Communication Interface')
        commSelectionBox.setObjectName('commInterface')
        # add new config input fields
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.commTypeCombo = QComboBox()
        self.commTypeCombo.setFont(fixedWidthFont)
        self.commTypeCombo.setMaximumWidth(220)
        boxLayout = QVBoxLayout()
        boxLayout.addWidget(self.commTypeCombo)
        commSelectionBox.setLayout(boxLayout)
        vLayout = QFormLayout()
        vLayout.addWidget(commSelectionBox)

        self.configsLayout = QVBoxLayout()
        self.configsBox = QGroupBox('')
        self.configsBox.setLayout(self.configsLayout)
        vLayout.addWidget(self.configsBox)

        self.setLayout(vLayout)
        self.resize(700, 500)
        #self.setStyleSheet('QGroupBox#commInterface { border: 1px solid black; border-radius: 4px; padding:15px;} QGroupBox::title#commInterface {background-color:transparent; padding-left:25px; padding-top:5px;} ')

        self.rosConfigsUI = RosConfigDialog('ROS Communication')
        self.rosConfigsUI.configChanged.connect(self.configChangedHandler)
        self.configsLayout.addWidget(self.rosConfigsUI)
        self.rosConfigsUI.setVisible(False)
        self.jderobotCommConfigsUI = JdeRobotCommConfigDialog('JdeRobot Communication')
        self.jderobotCommConfigsUI.configChanged.connect(self.configChangedHandler)
        self.configsLayout.addWidget(self.jderobotCommConfigsUI)
        self.jderobotCommConfigsUI.setVisible(True)

        self.rosConfig = None
        self.jdeRobotCommConfig = None

        self.commTypeCombo.addItem('JdeRobot Communication', 'jderobotcomm')
        self.commTypeCombo.addItem('ROS Node', 'ros')
        self.commTypeCombo.currentIndexChanged.connect(self.commTypeComboChanged)

        if config is not None:
            if config.type == ROS:
                self.rosConfig = config
                self.commTypeCombo.setCurrentIndex(1)
                self.loadRosConfigs()
            elif config.type == JDEROBOTCOMM:
                self.jdeRobotCommConfig = config
                self.commTypeCombo.setCurrentIndex(0)
                self.loadJdeRobotCommConfigs()
        else:
            self.loadJdeRobotCommConfigs()



    def commTypeComboChanged(self):
        if self.commTypeCombo.currentData() == 'ros':
            self.loadRosConfigs()
        elif self.commTypeCombo.currentData() == 'jderobotcomm':
            self.loadJdeRobotCommConfigs()


    def loadRosConfigs(self):
        self.type = ROS
        self.jderobotCommConfigsUI.setVisible(False)
        self.rosConfigsUI.setVisible(True)
        if self.rosConfig is None:
            self.rosConfig = RosConfig()
        self.rosConfigsUI.setConfig(self.rosConfig)
        self.configChanged.emit()

    def loadJdeRobotCommConfigs(self):
        self.type = JDEROBOTCOMM
        self.rosConfigsUI.setVisible(False)
        self.jderobotCommConfigsUI.setVisible(True)
        if self.jdeRobotCommConfig is None:
            self.jdeRobotCommConfig = JdeRobotConfig()
        self.jderobotCommConfigsUI.setConfig(self.jdeRobotCommConfig)
        self.configChanged.emit()

    def configChangedHandler(self):
        self.configChanged.emit()

    def getConfig(self):
        if self.type == ROS:
            return self.rosConfig
        elif self.type == JDEROBOTCOMM:
            return self.jdeRobotCommConfig

if __name__ == '__main__':
    app = QApplication(sys.argv)
    config = JdeRobotConfig()
    dialog = ConfigDialog('Config', config)
    dialog.exec_()