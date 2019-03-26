#!/usr/bin/python3

# General imports
import sys
import config
import rospy

# Practice imports
from gui.GUI import MainWindow
from gui.threadGUI import ThreadGUI
from MyAlgorithm import MyAlgorithm
from PyQt5.QtWidgets import QApplication
from interfaces.camera import ListenerCamera
from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d

if __name__ == "__main__":

    cfg = config.load(sys.argv[1])
    ymlNode = cfg.getProperty('Stop')
    node = rospy.init_node(ymlNode["NodeName"], anonymous=True)

    # ------------ M O T O R S ----------------------------------
    print("Publishing "+  "Stop.Motors" + " with ROS messages")
    topicM = cfg.getProperty("Stop.Motors"+".Topic")
    maxW = cfg.getPropertyWithDefault("Stop.Motors"+".maxW", 0.5)
    if not maxW:
        maxW = 0.5
        print ("Stop.Motors"+".maxW not provided, the default value is used: "+ repr(maxW))

    maxV = cfg.getPropertyWithDefault("Stop.Motors"+".maxV", 5)
    if not maxV:
        maxV = 5
        print ("Stop.Motors"+".maxV not provided, the default value is used: "+ repr(maxV))
    
    # motors = PublisherMotors("/opel/cmd_vel", maxV, maxW)
    motors = PublisherMotors(topicM, maxV, maxW)


    # ----------------- P O S E     3 D -------------------------------------
    print("Receiving " + "Stop.Pose3D" + " from ROS messages")
    topicP = cfg.getProperty("Stop.Pose3D"+".Topic")
    pose3d = ListenerPose3d(topicP)
    # pose3d = ListenerPose3d("/opel/odom")


    # -------- C A M E R A C E N T R A L --------------------------------------
    print("Receiving " + "Stop.CameraC" + "  CameraData from ROS messages")
    topicCameraC  = cfg.getProperty("Stop.CameraC"+".Topic")
    cameraC = ListenerCamera(topicCameraC)
    # cameraC = ListenerCamera("/opel/cameraC/image_raw")

    # -------- C A M E R A L E F T --------------------------------------------
    print("Receiving " + "Stop.CameraL" + "  CameraData from ROS messages")
    topicCameraL  = cfg.getProperty("Stop.CameraL"+".Topic")
    cameraL = ListenerCamera(topicCameraL)
    # cameraL = ListenerCamera("/opel/cameraL/image_raw")

    # -------- C A M E R A R I G H T ------------------------------------------
    print("Receiving " + "Stop.CameraR" + "  CameraData from ROS messages")
    topicCameraR  = cfg.getProperty("Stop.CameraR"+".Topic")
    cameraR = ListenerCamera(topicCameraR)
    # cameraR = ListenerCamera("/opel/cameraR/image_raw")

    print("Starting movement of dummy cars")
    motorsDummy1 = PublisherMotors("/dummy1/cmd_vel", 4, 0.3)
    motorsDummy2 = PublisherMotors("/dummy2/cmd_vel", 4, 0.3)

    motorsDummy1.sendV(2.5)
    motorsDummy2.sendV(3)

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
