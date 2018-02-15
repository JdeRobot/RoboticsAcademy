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
from PyQt5.QtWidgets import QDialog, \
    QLineEdit, QPushButton, \
    QWidget, QApplication, QLabel, QGridLayout
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFontDatabase


class LibrariesDialog(QDialog):
    librariesChanged = pyqtSignal(list)

    def __init__(self, name, libraries):
        super(QDialog, self).__init__()
        self.setWindowTitle(name)
        self.resize(300, 100)
        self.libraries = libraries
        self.libraryNameEdit = None
        self.addButton = None

        self.drawWindow()

    def drawWindow(self):
        if self.layout() is not None:
            tempWidget = QWidget()
            tempWidget.setLayout(self.layout())

        gridLayout = QGridLayout()

        # add header
        gridLayout.addWidget(QLabel('Libraries'), 0, 0)
        gridLayout.addWidget(QLabel(''), 0, 1)

        # add new library edit box
        self.libraryNameEdit = QLineEdit()
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.libraryNameEdit.setFont(fixedWidthFont)
        gridLayout.addWidget(self.libraryNameEdit, 1, 0)
        self.addButton = QPushButton('Add')
        self.addButton.clicked.connect(self.addClicked)
        gridLayout.addWidget(self.addButton, 1, 1)

        self.buttons = {}

        row = 2
        for lib in self.libraries:
            gridLayout.addWidget(QLabel(lib), row, 0)
            deleteButton = QPushButton()
            deleteButton.setObjectName(lib)
            deleteButton.setText('Delete')
            deleteButton.clicked.connect(self.deleteButtonClicked)
            gridLayout.addWidget(deleteButton, row, 1)
            row += 1
            self.buttons[deleteButton] = lib

        self.resize(300, 100)
        self.setLayout(gridLayout)


    def deleteButtonClicked(self):
        self.libraries.remove(self.sender().objectName())
        self.drawWindow()
        self.librariesChanged.emit(self.libraries)

    def addClicked(self):
        self.libraries.append(self.libraryNameEdit.text())
        self.drawWindow()
        self.librariesChanged.emit(self.libraries)



if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = LibrariesDialog('Libraries', ['okan'])
    dialog.exec_()





