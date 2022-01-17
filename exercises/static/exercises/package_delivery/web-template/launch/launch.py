#!/usr/bin/env python3

import stat
import rospy
from os import lstat
from subprocess import Popen, PIPE



DRI_PATH = "/dev/dri/card0"
EXERCISE = "package_delivery"
TIMEOUT = 30



# Check if acceleration can be enabled
def check_device(device_path):
    try:
        return stat.S_ISCHR(lstat(device_path)[stat.ST_MODE])
    except:
        return False



# Spawn new process
def spawn_process(args, insert_vglrun=False):
    if insert_vglrun:
        args.insert(0, "vglrun")
    process = Popen(args, stdout=PIPE, bufsize=1, universal_newlines=True)
    return process



class Test():
    def gazebo(self):
        rospy.logwarn("[GAZEBO] Performing checks")
        try:
            rospy.wait_for_service("/gazebo/get_model_properties", TIMEOUT)
        except rospy.ROSException:
            rospy.logwarn("[GAZEBO] Check timeout exceeded")


    def px4(self):
        rospy.logwarn("[PX4-SITL] Performing checks")
        start_time = rospy.get_time()
        args = ["./PX4-Autopilot/build/px4_sitl_default/bin/px4-commander", "check"]
        while rospy.get_time() - start_time < TIMEOUT:
            process = spawn_process(args, insert_vglrun=False)
            with process.stdout:
                for line in iter(process.stdout.readline, ''):
                    if ("Prearm check: OK" in line):
                        return
            rospy.sleep(2)
        rospy.logwarn("[PX4] Check timeout exceeded")


    def mavros(self, ns=""):
        rospy.logwarn("[MAVROS] Performing checks")
        try:
            rospy.wait_for_service(ns + "/mavros/cmd/arming", TIMEOUT)
        except rospy.ROSException:
            rospy.logwarn("[MAVROS] Check timeout exceeded")



class Launch():
    def __init__(self):
        self.test = Test()
        self.acceleration_enabled = check_device(DRI_PATH)

        # Start roscore
        args = ["/opt/ros/noetic/bin/roscore"]
        spawn_process(args, insert_vglrun=False)

        rospy.init_node("launch", anonymous=True)


    def start(self):
        ######## LAUNCH GAZEBO ########
        args = [
                    "/opt/ros/noetic/bin/roslaunch", 
                    "/RoboticsAcademy/exercises/" + EXERCISE + "/web-template/launch/gazebo.launch", 
                    "--wait", 
                    "--log"
                ]
        spawn_process(args, insert_vglrun=self.acceleration_enabled)
        self.test.gazebo()


        ######## LAUNCH PX4 ########
        args = [
                    "/opt/ros/noetic/bin/roslaunch", 
                    "/RoboticsAcademy/exercises/" + EXERCISE + "/web-template/launch/px4.launch", 
                    "--log"
                ]
        spawn_process(args, insert_vglrun=self.acceleration_enabled)
        self.test.px4()


        ######## LAUNCH MAVROS ########
        args = [
                    "/opt/ros/noetic/bin/roslaunch", 
                    "/RoboticsAcademy/exercises/" + EXERCISE + "/web-template/launch/mavros.launch", 
                    "--log"
                ]
        spawn_process(args, insert_vglrun=self.acceleration_enabled)
        self.test.mavros()



if __name__ == "__main__":
    launch = Launch()
    launch.start()
    
    with open("/status.txt", "w") as f:
            f.write("done")
