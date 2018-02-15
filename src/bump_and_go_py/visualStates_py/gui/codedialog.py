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
from PyQt5.QtWidgets import QDialog, QTextEdit, QPushButton, QVBoxLayout, \
    QWidget, QHBoxLayout, QApplication, QRadioButton
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFontDatabase, QColor, QFontMetrics
from PyQt5.Qsci import QsciScintilla, QsciLexerPython, QsciLexerCPP

class CodeDialog(QDialog):
    codeChanged = pyqtSignal('QString')

    def __init__(self, name, currentValue):
        super(QDialog, self).__init__()
        self.setWindowTitle(name)
        self.resize(800, 600)
        self.codeEdit = QsciScintilla()
        self.codeEdit.setText(currentValue)
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.codeEdit.setFont(fixedWidthFont)
        fontmetrics = QFontMetrics(fixedWidthFont)
        self.codeEdit.setMarginWidth(0, fontmetrics.width("000"))
        self.codeEdit.setMarginLineNumbers(0, True)
        self.codeEdit.setMarginsBackgroundColor(QColor("#cccccc"))

        self.codeEdit.setBraceMatching(QsciScintilla.SloppyBraceMatch)
        self.codeEdit.setCaretLineVisible(True)
        self.codeEdit.setCaretLineBackgroundColor(QColor("#ffe4e4"))
        lexer = QsciLexerPython()
        lexer.setDefaultFont(fixedWidthFont)
        self.codeEdit.setLexer(lexer)
        self.codeEdit.SendScintilla(QsciScintilla.SCI_SETHSCROLLBAR, 0)
        self.codeEdit.setUtf8(True)

        self.codeEdit.setTabWidth(4)
        self.codeEdit.setIndentationsUseTabs(True)
        self.codeEdit.setIndentationGuides(True)
        self.codeEdit.setTabIndents(True)
        self.codeEdit.setAutoIndent(True)

        self.cancelButton = QPushButton('Cancel')
        self.cancelButton.clicked.connect(self.cancel)
        self.acceptButton = QPushButton('Accept')
        self.acceptButton.clicked.connect(self.accept)

        self.pythonButton = QRadioButton('Python')
        self.pythonButton.setChecked(True)
        self.pythonButton.clicked.connect(self.pythonClicked)
        self.cppButton = QRadioButton('C++')
        self.cppButton.clicked.connect(self.cppClicked)

        hLayout0 = QHBoxLayout()
        hLayout0.addWidget(self.pythonButton)
        hLayout0.addWidget(self.cppButton)
        container0 = QWidget()
        container0.setLayout(hLayout0)

        verticalLayout = QVBoxLayout()
        verticalLayout.addWidget(container0)
        verticalLayout.addWidget(self.codeEdit)

        container = QWidget()
        hLayout =QHBoxLayout()
        hLayout.addWidget(self.cancelButton)
        hLayout.addWidget(self.acceptButton)
        container.setLayout(hLayout)

        verticalLayout.addWidget(container)
        self.setLayout(verticalLayout)

        self.language = 'python'

    def cancel(self):
        self.close()

    def accept(self):
        self.codeChanged.emit(self.codeEdit.text())
        self.close()

    def pythonClicked(self):
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        lexer = QsciLexerPython()
        lexer.setDefaultFont(fixedWidthFont)
        self.codeEdit.setLexer(lexer)
        self.language = 'python'

    def cppClicked(self):
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        lexer = QsciLexerCPP()
        lexer.setDefaultFont(fixedWidthFont)
        self.codeEdit.setLexer(lexer)
        self.language = 'cpp'

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = CodeDialog('Rename', 'Hello World')
    dialog.exec_()
