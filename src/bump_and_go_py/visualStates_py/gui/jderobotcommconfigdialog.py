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
from gui.interfaces import Interfaces
from PyQt5.QtWidgets import QDialog, QGroupBox, \
    QLineEdit, QVBoxLayout, QHBoxLayout, QPushButton, \
    QWidget, QApplication, QLabel, QGridLayout, QComboBox
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFontDatabase
from gui.config import JdeRobotConfig
from gui.config import Config

class JdeRobotCommConfigDialog(QDialog):
    configChanged = pyqtSignal()

    def __init__(self, name):
        super(QDialog, self).__init__()
        self.setWindowTitle(name)
        self.config = None

        self.gridLayout = QGridLayout()

        # add header
        self.gridLayout.addWidget(QLabel('Server Type'), 0, 0)
        self.gridLayout.addWidget(QLabel('Name'), 0, 1)
        self.gridLayout.addWidget(QLabel('Topic'), 0, 2)
        self.gridLayout.addWidget(QLabel('Proxy Name'), 0, 3)
        self.gridLayout.addWidget(QLabel('IP'), 0, 4)
        self.gridLayout.addWidget(QLabel('Port'), 0, 5)
        self.gridLayout.addWidget(QLabel('Interface'), 0, 6)
        self.gridLayout.addWidget(QLabel(''), 0, 7)

        # add new config input fields
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.serverTypeCombo = QComboBox()
        self.serverTypeCombo.setFont(fixedWidthFont)
        self.gridLayout.addWidget(self.serverTypeCombo, 1, 0)
        self.nameEdit = QLineEdit()
        self.nameEdit.setFont(fixedWidthFont)
        self.gridLayout.addWidget(self.nameEdit, 1, 1)
        self.topicEdit = QLineEdit()
        self.topicEdit.setFont(fixedWidthFont)
        self.topicEdit.setEnabled(False)
        self.gridLayout.addWidget(self.topicEdit, 1, 2)
        self.proxyNameEdit = QLineEdit()
        self.proxyNameEdit.setFont(fixedWidthFont)
        self.gridLayout.addWidget(self.proxyNameEdit, 1, 3)
        self.ipEdit = QLineEdit()
        self.ipEdit.setFont(fixedWidthFont)
        self.gridLayout.addWidget(self.ipEdit, 1, 4)
        self.portEdit = QLineEdit()
        self.portEdit.setFont(fixedWidthFont)
        self.gridLayout.addWidget(self.portEdit, 1, 5)
        self.interfaceCombo = QComboBox()
        self.gridLayout.addWidget(self.interfaceCombo, 1, 6)
        self.addButton = QPushButton('Add')
        self.gridLayout.addWidget(self.addButton, 1, 7)
        self.addButton.clicked.connect(self.addClicked)

        self.rowCount = 2

        # add server types to the combobox
        self.serverTypeCombo.addItem('ICE', 'ice')
        self.serverTypeCombo.addItem('ROS', 'ros')
        self.serverTypeCombo.currentIndexChanged.connect(self.serverTypeChanged)

        # add interfaces to the combobox
        interfaces = Interfaces.getInterfaces()
        for interfaceName in interfaces:
            self.interfaceCombo.addItem(interfaceName, interfaceName)

        self.resize(700, 100)
        self.setLayout(self.gridLayout)

    def clearAll(self):
        deleteList = []
        for i in range(self.rowCount):
            if i == 0 or i == 1:
                continue
            else:
                for j in range(8):
                    item = self.gridLayout.itemAtPosition(i, j)
                    deleteList.append(item)

        for item in deleteList:
            self.gridLayout.removeItem(item)
            item.widget().setParent(None)

        self.rowCount = 2


    def setConfig(self, config):
        self.config = config
        self.clearAll()
        if self.config is not None:
            for interface in self.config.getInterfaces():
                interface['id'] = self.rowCount
                self.addConfigRow(interface)


    def addConfigRow(self, configData):
        self.gridLayout.addWidget(QLabel(configData['serverType']), self.rowCount, 0)
        self.gridLayout.addWidget(QLabel(configData['name']), self.rowCount, 1)
        self.gridLayout.addWidget(QLabel(configData['topic']), self.rowCount, 2)
        self.gridLayout.addWidget(QLabel(configData['proxyName']), self.rowCount, 3)
        self.gridLayout.addWidget(QLabel(configData['ip']), self.rowCount, 4)
        self.gridLayout.addWidget(QLabel(configData['port']), self.rowCount, 5)
        self.gridLayout.addWidget(QLabel(configData['interface']), self.rowCount, 6)
        deleteButton = QPushButton('Delete')
        deleteButton.clicked.connect(self.deleteClicked)
        # we will find the item to be deleted based on the index on the config list
        deleteButton.setObjectName(str(self.rowCount))
        self.gridLayout.addWidget(deleteButton, self.rowCount, 7)
        self.rowCount += 1

    def serverTypeChanged(self):
        if self.serverTypeCombo.currentData() == 'ros':
            self.topicEdit.setEnabled(True)
            self.proxyNameEdit.setEnabled(False)
            self.ipEdit.setEnabled(False)
            self.portEdit.setEnabled(False)
        elif self.serverTypeCombo.currentData() == 'ice':
            self.topicEdit.setEnabled(False)
            self.proxyNameEdit.setEnabled(True)
            self.ipEdit.setEnabled(True)
            self.portEdit.setEnabled(True)
        self.configChanged.emit()

    def deleteClicked(self):
        if self.config is not None:
            id = int(self.sender().objectName())
            self.config.removeInterface(id)
            # reset the config to redraw all configs
            self.setConfig(self.config)
            self.configChanged.emit()

    def addClicked(self):
        configData = {}
        configData['serverType'] = self.serverTypeCombo.currentData()
        configData['name'] = self.nameEdit.text()
        configData['topic'] = self.topicEdit.text()
        configData['proxyName'] = self.proxyNameEdit.text()
        configData['ip'] = self.ipEdit.text()
        configData['port'] = self.portEdit.text()
        configData['interface'] = self.interfaceCombo.currentData()

        self.nameEdit.setText('')
        self.topicEdit.setText('')
        self.proxyNameEdit.setText('')
        self.ipEdit.setText('')
        self.portEdit.setText('')

        if self.config is not None:
            configData['id'] = self.rowCount
            self.config.addInterface(configData)
            self.addConfigRow(configData)
            self.configChanged.emit()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = JdeRobotCommConfigDialog('Config')
    config = JdeRobotConfig()
    dialog.setConfig(config)
    dialog.exec_()