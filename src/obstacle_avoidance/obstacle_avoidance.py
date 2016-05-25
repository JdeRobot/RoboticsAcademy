#!/usr/bin/python

__author__ = 'frivas'


import sys
from PyQt4 import QtCore, QtGui
from gui.GUI import MainWindow
from gui.threadGUI import ThreadGUI
from sensors.sensor import Sensor
from sensors.threadSensor import ThreadSensor
from MyAlgorithm import MyAlgorithm




if __name__ == "__main__":
    sensor = Sensor()
    algorithm=MyAlgorithm(sensor)

    app = QtGui.QApplication(sys.argv)
    myGUI = MainWindow()
    myGUI.setSensor(sensor)
    myGUI.setAlgorithm(algorithm)
    myGUI.show()

    t1 = ThreadSensor(sensor,algorithm)
    t1.daemon=True
    t1.start()

    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()


    sys.exit(app.exec_())
