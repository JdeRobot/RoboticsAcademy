#!/usr/bin/python3

# General imports
import sys

# Practice imports
from gui.GUI import MainWindow
from gui.threadGUI import ThreadGUI
from MyAlgorithm import MyAlgorithm
from PyQt5.QtWidgets import QApplication
from interfaces.camera import ListenerCamera
from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d

if __name__ == "__main__":

    cameraC = ListenerCamera("/opel/cameraC/image_raw")
    cameraL = ListenerCamera("/opel/cameraL/image_raw")
    cameraR = ListenerCamera("/opel/cameraR/image_raw")
    motors = PublisherMotors("/opel/cmd_vel", 4, 0.3)
    pose3d = ListenerPose3d("/opel/odom")

    algorithm = MyAlgorithm(pose3d, cameraC, cameraL, cameraR, motors)

    app = QApplication(sys.argv)
    myGUI = MainWindow()
    myGUI.setMotors(motors)
    myGUI.setCameraC(cameraC)
    myGUI.setCameraL(cameraL)
    myGUI.setCameraR(cameraR)
    myGUI.setPose3D(pose3d)
    myGUI.setAlgorithm(algorithm)
    myGUI.show()

    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()


    sys.exit(app.exec_())
