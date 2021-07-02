#!/usr/bin/env python3

import os
import stat
import time
import rospy
import fcntl
import subprocess

# Function to check if a device exists
def check_device(device_path):
    try:
        return stat.S_ISCHR(os.lstat(device_path)[stat.ST_MODE])
    except:
        return False

DRI_PATH = "/dev/dri/card0"
ACCELERATION_ENABLED = check_device(DRI_PATH)
EXERCISE = "visual_lander"


class Tests():
    def test_px4(self):
        rospy.logwarn("[PX4-SITL] Performing checks")
        while True:
            args = ["./PX4-Autopilot/build/px4_sitl_default/bin/px4-commander", "check"]
            process = subprocess.Popen(args, stdout=subprocess.PIPE, bufsize=1, universal_newlines=True)
            with process.stdout:
                for line in iter(process.stdout.readline, ''):
                    if ("Prearm check: OK" in line):
                        return
            time.sleep(2)

    def test_mavros(self, ns=""):
        rospy.logwarn("[MAVROS] Performing checks")
        rospy.wait_for_service(ns + "/mavros/cmd/arming", 30)

    def test_gazebo(self):
        rospy.logwarn("[GAZEBO] Performing checks")
        rospy.wait_for_service("/gazebo/get_model_properties", 30)


class Launch(Tests):
    def __init__(self):
        args = ["/opt/ros/noetic/bin/roscore"]
        self.run(args, insert_roslaunch=False, insert_vglrun=False) #start roscore
        rospy.init_node("launch", anonymous=True)

    def run(self, args, insert_roslaunch=True, insert_vglrun=True):
        if insert_roslaunch: args.insert(0, "/opt/ros/noetic/bin/roslaunch")
        if insert_vglrun and ACCELERATION_ENABLED: args.insert(0, "vglrun")
        subprocess.Popen(args, stdout=subprocess.PIPE, bufsize=1, universal_newlines=True)

    def finished(self):
        while True:
            try:
                with open("/status.txt", "w") as f:
                    fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB) #lock
                    f.write("done")
                    fcntl.flock(f, fcntl.LOCK_UN) #unlock
                    break
            except:
                time.sleep(0.05)

    def main(self):
        try:
            args1 = ["/RoboticsAcademy/exercises/" + EXERCISE + "/web-template/launch/gazebo.launch", "--wait"]
            args2 = ["/RoboticsAcademy/exercises/" + EXERCISE + "/web-template/launch/px4.launch"]
            args3 = ["/RoboticsAcademy/exercises/" + EXERCISE + "/web-template/launch/mavros.launch"]

            self.run(args1)     #launch gazebo
            self.test_gazebo()
            self.run(args2)     #launch px4
            self.test_px4()
            self.run(args3)     #launch mavros
            self.test_mavros()
            self.finished()

        except Exception as e:
            rospy.logerr(e)


if __name__ == "__main__":
    launch = Launch()
    launch.main()
