#!/usr/bin/python3
#
#  Copyright (C) 1997-2016 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Eduardo Perdices <eperdices@gsyc.es>
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#

import sys, os, config
import rospy

from gui.GUI import MainWindow
from gui.threadGUI import ThreadGUI
from MyAlgorithm import MyAlgorithm
from PyQt5.QtWidgets import QApplication
from interfaces.laser import ListenerLaser
from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d

if __name__ == "__main__":

    cfg = config.load(sys.argv[1])
    ymlNode = cfg.getProperty('Autopark')
    node = rospy.init_node(ymlNode["NodeName"], anonymous=True)

    # ------------ M O T O R S ----------------------------------
    print("Publishing "+  "Autopark.Motors" + " with ROS messages")
    topicM = cfg.getProperty("Autopark.Motors"+".Topic")
    maxW = cfg.getPropertyWithDefault("Autopark.Motors"+".maxW", 0.5)
    if not maxW:
        maxW = 0.5
        print ("Autopark.Motors"+".maxW not provided, the default value is used: "+ repr(maxW))

    maxV = cfg.getPropertyWithDefault("Autopark.Motors"+".maxV", 20)
    if not maxV:
        maxV = 5
        print ("Autopark.Motors"+".maxV not provided, the default value is used: "+ repr(maxV))
    
    # motors = PublisherMotors("/taxi_holo_laser/cmd_vel", 20, 0.5)
    motors = PublisherMotors(topicM, maxV, maxW)

    # ----------------- P O S E     3 D -------------------------------------
    print("Receiving " + "Autopark.Pose3D" + " from ROS messages")
    topicP = cfg.getProperty("Autopark.Pose3D"+".Topic")
    pose3d = ListenerPose3d(topicP)
    # pose3d = ListenerPose3d("/taxi_holo_laser/odom")

    # -------- L A S E R O N E --------------------------------------
    print("Receiving " + "Autopark.Laser1" + "  CameraData from ROS messages")
    topicLaser1  = cfg.getProperty("Autopark.Laser1"+".Topic")
    laser1 = ListenerLaser(topicLaser1)
    # laser1 = ListenerLaser("/taxi_holo_laser/laser1/scan")

    # -------- L A S E R T W O --------------------------------------
    print("Receiving " + "Autopark.Laser1" + "  CameraData from ROS messages")
    topicLaser2  = cfg.getProperty("Autopark.Laser2"+".Topic")
    laser2 = ListenerLaser(topicLaser2)
    # laser2 = ListenerLaser("/taxi_holo_laser/laser2/scan")

    # -------- L A S E R T H R E E --------------------------------------
    print("Receiving " + "Autopark.Laser1" + "  CameraData from ROS messages")
    topicLaser3  = cfg.getProperty("Autopark.Laser3"+".Topic")
    laser3 = ListenerLaser(topicLaser3)
    # laser3 = ListenerLaser("/taxi_holo_laser/laser3/scan")

    algorithm=MyAlgorithm(pose3d, laser1, laser2, laser3, motors)

    app = QApplication(sys.argv)
    myGUI = MainWindow()
    myGUI.setMotors(motors)
    myGUI.setPose3D(pose3d)
    myGUI.setLaser1(laser1)
    myGUI.setLaser2(laser2)
    myGUI.setLaser3(laser3)
    myGUI.setAlgorithm(algorithm)
    myGUI.show()


    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()


    id = app.exec_()
    os._exit(id)
