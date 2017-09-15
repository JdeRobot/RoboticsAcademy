import sys
from PyQt5.QtWidgets import QApplication
from gui.GUI import MainWindow
from gui.threadGUI import ThreadGUI
from parallelIce.motors import Motors
from parallelIce.pose3dClient import Pose3DClient
from parallelIce.laserClient import LaserClient
from parallelIce.bumperClient import BumperClient
import easyiceconfig as EasyIce
from MyAlgorithm import MyAlgorithm


if __name__ == "__main__":
    ic = EasyIce.initialize(sys.argv)
    motors = Motors (ic, "VacuumCleaner.Motors")
    pose3d = Pose3DClient(ic, "VacuumCleaner.Pose3D", True)
    laser = LaserClient(ic, "VacuumCleaner.Laser", True)
    bumper = BumperClient(ic, "VacuumCleaner.Bumper", True)
    algorithm=MyAlgorithm(pose3d, motors,laser, bumper)


    app = QApplication(sys.argv)
    myGUI = MainWindow()
    myGUI.setMotors(motors)
    myGUI.setPose3D(pose3d)
    myGUI.setLaser(laser)
    myGUI.setBumper(bumper)
    myGUI.setAlgorithm(algorithm)
    myGUI.show()


    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()


    sys.exit(app.exec_())
