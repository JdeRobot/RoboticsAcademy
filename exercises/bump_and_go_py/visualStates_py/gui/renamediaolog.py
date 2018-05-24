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
from PyQt5.QtWidgets import QDialog, QLineEdit, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QApplication
from PyQt5.QtCore import pyqtSignal

class RenameDialog(QDialog):
    nameChanged = pyqtSignal('QString')

    def __init__(self, name, currentValue):
        super(QDialog, self).__init__()
        self.setWindowTitle(name)
        self.nameEdit = QLineEdit()
        self.nameEdit.setText(currentValue)
        self.cancelButton = QPushButton('Cancel')
        self.cancelButton.clicked.connect(self.cancel)
        self.acceptButton = QPushButton('Accept')
        self.acceptButton.clicked.connect(self.accept)

        verticalLayout = QVBoxLayout()
        verticalLayout.addWidget(self.nameEdit)

        container = QWidget()
        hLayout =QHBoxLayout()
        hLayout.addWidget(self.cancelButton)
        hLayout.addWidget(self.acceptButton)
        container.setLayout(hLayout)

        verticalLayout.addWidget(container)
        self.setLayout(verticalLayout)

    def cancel(self):
        self.close()

    def accept(self):
        self.nameChanged.emit(self.nameEdit.text())
        self.close()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = RenameDialog('Rename', 'Hello World')
    dialog.exec_()
