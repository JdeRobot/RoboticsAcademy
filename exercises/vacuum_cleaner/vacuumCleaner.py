import sys
import config
import comm
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

    cfg = config.load(sys.argv[1])

    #starting comm
    jdrc= comm.init(cfg, 'VacuumCleaner')

    motors = jdrc.getMotorsClient("VacuumCleaner.Motors")
    pose3d = jdrc.getPose3dClient("VacuumCleaner.Pose3D")
    laser = jdrc.getLaserClient("VacuumCleaner.Laser").hasproxy()
    bumper = jdrc.getBumperClient("VacuumCleaner.Bumper")
    algorithm=MyAlgorithm(pose3d, motors,laser, bumper)

    app = QApplication(sys.argv)
    myGUI = MainWindow(pose3d)
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
