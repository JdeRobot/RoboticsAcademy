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
    QWidget, QApplication, QLabel, QGridLayout, QComboBox, \
    QFormLayout, QTabWidget, QPlainTextEdit
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFontDatabase
from gui.config import RosConfig

class RosConfigDialog(QDialog):
    configChanged = pyqtSignal()

    # use this for editable filtering of message types http://www.qtcentre.org/threads/23143-Combobox-entries-filter-as-I-type

    def __init__(self, name):
        super(QDialog, self).__init__()
        self.setWindowTitle(name)
        self.tabWidget = None

        self.config = None
        self.packageTab = None
        self.topicsTab = None

        self.tabWidget = QTabWidget()
        mainLayout = QFormLayout()
        mainLayout.addWidget(self.tabWidget)
        self.packageTab = PackageTab()
        self.packageTab.configChanged.connect(self.configChangedHandler)
        self.tabWidget.addTab(self.packageTab, 'Package')
        self.topicsTab = TopicsTab()
        self.topicsTab.configChanged.connect(self.configChangedHandler)
        self.tabWidget.addTab(self.topicsTab, 'Topics')
        self.resize(700, 100)
        self.setLayout(mainLayout)

    def setConfig(self, config):
        self.config = config
        self.packageTab.setConfig(self.config)
        self.topicsTab.setConfig(self.config)

    def configChangedHandler(self):
        self.configChanged.emit()


class PackageTab(QWidget):
    configChanged = pyqtSignal()
    def __init__(self):
        super(QWidget, self).__init__()
        layout = QFormLayout()
        self.setLayout(layout)

        self.buildDependencies = QPlainTextEdit()
        self.buildDependencies.textChanged.connect(self.buildDependenciesChanged)
        self.buildDependencies.setMinimumHeight(70)
        layout.addRow('Build Dependencies', self.buildDependencies)

        self.runDependencies = QPlainTextEdit()
        self.runDependencies.textChanged.connect(self.runDependenciesChanged)
        self.runDependencies.setMinimumHeight(70)
        layout.addRow('Run Dependencies', self.runDependencies)

    def buildDependenciesChanged(self):
        if self.config is not None:
            self.config.setBuildDependencies(self.buildDependencies.toPlainText())
            self.configChanged.emit()

    def runDependenciesChanged(self):
        if self.config is not None:
            self.config.setRunDependencies(self.runDependencies.toPlainText())
            self.configChanged.emit()

    def setConfig(self, config):
        self.config = config
        self.buildDependencies.setPlainText(self.config.getBuildDependenciesAsText())
        self.runDependencies.setPlainText(self.config.getRunDependenciesAsText())



class TopicsTab(QWidget):
    configChanged = pyqtSignal()
    def __init__(self):
        super(QWidget, self).__init__()
        self.config = None
        self.count = 0
        self.topicRows = {}

        self.nameEdit = QLineEdit()
        self.dataTypeComboBox = QComboBox()
        self.fillDataTypes()
        self.opTypeComboBox = QComboBox()
        self.opTypeComboBox.addItem('sub', 'Subscribe')
        self.opTypeComboBox.addItem('pub', 'Publish')
        self.addButton = QPushButton('Add')
        self.addButton.clicked.connect(self.addClicked)

        self.mainLayout = QVBoxLayout()
        rowLayout = QHBoxLayout()
        rowLayout.addWidget(self.nameEdit)
        rowLayout.addWidget(self.dataTypeComboBox)
        rowLayout.addWidget(self.opTypeComboBox)
        rowLayout.addWidget(self.addButton)
        rowContainer = QWidget()
        rowContainer.setLayout(rowLayout)
        rowContainer.setObjectName('titleRow')
        self.mainLayout.addWidget(rowContainer)
        self.setLayout(self.mainLayout)


    def fillDataTypes(self):
        rosTypes = Interfaces.getRosMessageTypes()
        for type in rosTypes:
            concatType = type['typeDir'] + '/' + type['type']
            self.dataTypeComboBox.addItem(concatType, concatType)


    def addTopicRow(self, name, type, opType):
        rowLayout = QHBoxLayout()
        rowLayout.addWidget(QLabel(name))
        rowLayout.addWidget(QLabel(type))
        rowLayout.addWidget(QLabel(opType))
        removeButton = QPushButton('Remove')
        removeButton.clicked.connect(self.removeTopicClicked)
        removeButton.setObjectName(str(self.count))
        rowLayout.addWidget(removeButton)
        rowContainer = QWidget()
        rowContainer.setLayout(rowLayout)
        rowContainer.setObjectName('row' + str(self.count))
        self.mainLayout.addWidget(rowContainer)
        self.topicRows[self.count] = rowContainer
        self.count += 1


    def addClicked(self):
        if self.config is not None:
            self.config.addTopic(self.count, self.nameEdit.text(), self.dataTypeComboBox.currentData(), self.opTypeComboBox.currentData())
            self.addTopicRow(self.nameEdit.text(), self.dataTypeComboBox.currentData(), self.opTypeComboBox.currentData())
            self.nameEdit.setText('')
            self.configChanged.emit()

    def removeTopicClicked(self):
        if self.config is not None:
            itemToRemove = None
            for i in range(self.mainLayout.count()):
                if self.mainLayout.itemAt(i).widget().objectName() == 'row' + self.sender().objectName():
                    itemToRemove = self.mainLayout.itemAt(i)
                    break
            if itemToRemove is not None:
                self.mainLayout.removeItem(itemToRemove)
                itemToRemove.widget().setParent(None)
                self.mainLayout.update()
                self.configChanged.emit()
            self.config.removeTopic(int(self.sender().objectName()))
            del self.topicRows[int(self.sender().objectName())]


    def clearAllRows(self):
        clearList = []
        for i in range(self.mainLayout.count()):
            item = self.mainLayout.itemAt(i)
            if item.widget().objectName() != 'titleRow':
                clearList.append(item)

        for item in clearList:
            self.mainLayout.removeItem(item)
            item.widget().setParent(None)

        self.mainLayout.update()
        self.count = 0


    def setConfig(self, config):
        self.config = config
        self.clearAllRows()
        for topic in self.config.getTopics():
            topic['id'] = self.count
            self.addTopicRow(topic['name'], topic['type'], topic['opType'])


if __name__ == '__main__':
    app = QApplication(sys.argv)
    config = RosConfig()

    dialog = RosConfigDialog('Config')
    dialog.setConfig(config)

    dialog.exec_()