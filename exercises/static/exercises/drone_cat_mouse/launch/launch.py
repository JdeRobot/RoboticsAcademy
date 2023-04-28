#!/usr/bin/env python3

import stat
import rospy
import os
from subprocess import Popen, PIPE


# If DRI_NAME is not set by user, use card0
DRI_PATH = os.path.join("/dev/dri", os.environ.get("DRI_NAME", "card0"))
EXERCISE = "drone_cat_mouse"
TIMEOUT = 30
MAX_ATTEMPT = 2


# Check if acceleration can be enabled
def check_device(device_path):
    try:
        return stat.S_ISCHR(os.lstat(device_path)[stat.ST_MODE])
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
        rospy.logwarn("[GAZEBO] Launching")
        try:
            rospy.wait_for_service("/gazebo/get_model_properties", TIMEOUT)
            return True
        except rospy.ROSException:
            return False

    def px4(self, instance):
        rospy.logwarn("[PX4-SITL] Launching")
        start_time = rospy.get_time()
        args = ["./PX4-Autopilot/build/px4_sitl_default/bin/px4-commander",
                "--instance", str(instance), "check"]
        while rospy.get_time() - start_time < TIMEOUT:
            process = spawn_process(args, insert_vglrun=False)
            with process.stdout:
                for line in iter(process.stdout.readline, ''):
                    if ("Prearm check: OK" in line):
                        return True
            rospy.sleep(2)
        return False

    def mavros(self, ns=""):
        rospy.logwarn("[MAVROS] Launching")
        try:
            rospy.wait_for_service(ns + "/mavros/cmd/arming", TIMEOUT)
            return True
        except rospy.ROSException:
            return False


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
        args = ["/opt/ros/noetic/bin/roslaunch",
                "/RoboticsAcademy/exercises/static/exercises/" +
                EXERCISE + "/launch/gazebo.launch",
                "--wait",
                "--log"
                ]

        attempt = 1
        while True:
            spawn_process(args, insert_vglrun=self.acceleration_enabled)
            if self.test.gazebo() == True:
                break
            if attempt == MAX_ATTEMPT:
                rospy.logerr("[GAZEBO] Launch Failed")
                return
            attempt = attempt + 1

        ######## LAUNCH PX4_CAT (INSTANCE 0) ########
        args = ["/opt/ros/noetic/bin/roslaunch",
                "/RoboticsAcademy/exercises/static/exercises/" +
                EXERCISE + "/launch/px4_cat.launch",
                "--log"
                ]

        attempt = 1
        while True:
            spawn_process(args, insert_vglrun=self.acceleration_enabled)
            if self.test.px4("0") == True:
                break
            if attempt == MAX_ATTEMPT:
                rospy.logerr("[PX4_CAT] Launch Failed")
                return
            attempt = attempt + 1

        ######## LAUNCH MAVROS_CAT ########
        args = ["/opt/ros/noetic/bin/roslaunch",
                "/RoboticsAcademy/exercises/static/exercises/" +
                EXERCISE + "/launch/mavros_cat.launch",
                "--log"
                ]

        attempt = 1
        while True:
            spawn_process(args, insert_vglrun=self.acceleration_enabled)
            if self.test.mavros("/cat") == True:
                break
            if attempt == MAX_ATTEMPT:
                rospy.logerr("[MAVROS_CAT] Launch Failed")
                return
            attempt = attempt + 1

        ######## LAUNCH PX4_MOUSE  (INSTANCE 1) ########
        args = ["/opt/ros/noetic/bin/roslaunch",
                "/RoboticsAcademy/exercises/static/exercises/" +
                EXERCISE + "/launch/px4_mouse.launch",
                "--log"
                ]

        attempt = 1
        while True:
            spawn_process(args, insert_vglrun=self.acceleration_enabled)
            if self.test.px4("1") == True:
                break
            if attempt == MAX_ATTEMPT:
                rospy.logerr("[PX4_MOUSE] Launch Failed")
                return
            attempt = attempt + 1

        ######## LAUNCH MAVROS_MOUSE ########
        args = ["/opt/ros/noetic/bin/roslaunch",
                "/RoboticsAcademy/exercises/static/exercises/" +
                EXERCISE + "/launch/mavros_mouse.launch",
                "--log"
                ]

        attempt = 1
        while True:
            spawn_process(args, insert_vglrun=self.acceleration_enabled)
            if self.test.mavros("/mouse") == True:
                break
            if attempt == MAX_ATTEMPT:
                rospy.logerr("[MAVROS_MOUSE] Launch Failed")
                return
            attempt = attempt + 1


if __name__ == "__main__":
    launch = Launch()
    launch.start()

    with open("/drones_launch.log", "w") as f:
        f.write("success")
