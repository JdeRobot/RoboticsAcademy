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
from PyQt5.QtWidgets import QDialog, QGroupBox, \
    QLineEdit, QVBoxLayout, QHBoxLayout, QPushButton, \
    QWidget, QApplication
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFontDatabase


class TimerDialog(QDialog):
    timeChanged = pyqtSignal('int')

    def __init__(self, name, timeValue):
        super(QDialog, self).__init__()
        self.resize(300, 150)

        timeContainer = QGroupBox()
        timeContainer.setTitle('Time (ms)')

        self.lineEdit = QLineEdit()
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.lineEdit.setFont(fixedWidthFont)
        self.lineEdit.setText(str(timeValue))
        vLayout = QVBoxLayout()
        vLayout.addWidget(self.lineEdit)

        self.cancelButton = QPushButton()
        self.cancelButton.setText('Cancel')
        self.cancelButton.clicked.connect(self.cancelClicked)

        self.acceptButton = QPushButton()
        self.acceptButton.setText('Accept')
        self.acceptButton.clicked.connect(self.acceptClicked)

        buttonContainer = QWidget()
        hLayout = QHBoxLayout()
        hLayout.addWidget(self.cancelButton)
        hLayout.addWidget(self.acceptButton)
        buttonContainer.setLayout(hLayout)
        vLayout.addWidget(buttonContainer)
        timeContainer.setLayout(vLayout)

        vLayout2 = QVBoxLayout()
        vLayout2.addWidget(timeContainer)
        self.setLayout(vLayout2)

    def cancelClicked(self):
        self.close()

    def acceptClicked(self):
        #todo: make sure that provided value is integer
        self.timeChanged.emit(int(self.lineEdit.text()))
        self.close()
        pass

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = TimerDialog('Timer', 100)
    dialog.exec_()