#!/usr/bin/env python3

import os
import sys
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
EXERCISE = "drone_cat_mouse"


class Tests():
    def test_px4__(self, n=1):
        rospy.logwarn("[PX4-SITL] Performing checks")
        passed, failed = False, False
        while passed and failed:
            args = ["./PX4-Autopilot/build/px4_sitl_default/bin/px4-commander_tests", "check"]
            process = subprocess.Popen(args, stdout=subprocess.PIPE, bufsize=1, universal_newlines=True)
            with process.stdout:
                for line in iter(process.stdout.readline, ''):
                    if ("INFO  [commander_tests]   Tests passed :" in line):
                        passed = int(line.split()[-1]) == n
                    if ("INFO  [commander_tests]   Tests failed :" in line):
                        failed = int(line.split()[-1]) == 0
            time.sleep(2)

    def test_px4(self, output):
        rospy.logwarn("[PX4-SITL] Performing checks")
        while True:
            for line in output:
                if "INFO  [px4] Startup script returned successfully" in line:
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
        if len(sys.argv)>=2 and sys.argv[1] == "log":
            with open("/logs/launch.log", "a+") as f:
                subprocess.Popen(args, stdout=f, bufsize=4096, universal_newlines=True)
        else:
            process = subprocess.Popen(args, stdout=subprocess.PIPE, bufsize=1, universal_newlines=True)
            return process.stdout
        # subprocess.Popen(args, stdout=subprocess.PIPE, bufsize=1, universal_newlines=True)

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
            args1 = ["/RoboticsAcademy/exercises/" + EXERCISE + "/web-template/launch/gazebo.launch", "--wait", "--log"]
            args2 = ["/RoboticsAcademy/exercises/" + EXERCISE + "/web-template/launch/px4_cat.launch", "--log"]
            args3 = ["/RoboticsAcademy/exercises/" + EXERCISE + "/web-template/launch/mavros_cat.launch", "--log"]
            args4 = ["/RoboticsAcademy/exercises/" + EXERCISE + "/web-template/launch/px4_mouse.launch", "--log"]
            args5 = ["/RoboticsAcademy/exercises/" + EXERCISE + "/web-template/launch/mavros_mouse.launch", "--log"]

            self.run(args1)         #launch gazebo
            self.test_gazebo()
            output = self.run(args2)         #launch px4 cat
            self.test_px4(output)
            self.run(args3)         #launch mavros cat
            self.test_mavros("/cat")
            output = self.run(args4)         #launch px4 mouse
            self.test_px4(output)
            self.run(args5)         #launch mavros mouse
            self.test_mavros("/mouse")
            self.finished()

        except Exception as e:
            rospy.logerr(e)


if __name__ == "__main__":
    launch = Launch()
    launch.main()
