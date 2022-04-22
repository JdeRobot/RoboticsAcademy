#!/usr/bin/env python3

import os
import stat
import rospy
import subprocess


DRI_PATH = "/dev/dri/card0"
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
    process = subprocess.Popen(args, stdout=subprocess.PIPE, bufsize=1, universal_newlines=True)
    return process


# Function to find px4 sitl log files
def find_sitl_log(path, regex):
    for filename in os.listdir(path):
        if regex in filename:
            return filename


class Test():
    def gazebo(self):
        rospy.logwarn("[GAZEBO] Launching")
        try:
            rospy.wait_for_service("/gazebo/get_model_properties", TIMEOUT)
            return True
        except rospy.ROSException:
            return False

    # def px4(self, path, regex):
    #     rospy.logwarn("[PX4-SITL] Launching")
    #     start_time = rospy.get_time()
    #     while rospy.get_time() - start_time < TIMEOUT:
    #         rospy.sleep(1)
    #         filename = find_sitl_log(path, regex)  # None until sitl launchs
    #         if filename is None: continue
    #         with open(os.path.join(path, filename), "r") as f:
    #             for line in f:
    #                 if "INFO  [px4] Startup script returned successfully" in line:
    #                     return True
    #     return False

    def px4(self, instance):
        rospy.logwarn("[PX4-SITL] Launching")
        start_time = rospy.get_time()
        args = ["./PX4-Autopilot/build/px4_sitl_default/bin/px4-commander","--instance", instance, "check"]
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

        process = subprocess.run(["roslaunch-logs"], shell=True, stdout=subprocess.PIPE, encoding="utf-8")
        self.log_path = process.stdout[:-1]


    def start(self):
        ######## LAUNCH GAZEBO ########
        args = ["/opt/ros/noetic/bin/roslaunch", 
                "/RoboticsAcademy/exercises/" + EXERCISE + "/web-template/launch/gazebo.launch", 
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
                "/RoboticsAcademy/exercises/" + EXERCISE + "/web-template/launch/px4_cat.launch", 
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
                "/RoboticsAcademy/exercises/" + EXERCISE + "/web-template/launch/mavros_cat.launch", 
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
                "/RoboticsAcademy/exercises/" + EXERCISE + "/web-template/launch/px4_mouse.launch", 
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
                "/RoboticsAcademy/exercises/" + EXERCISE + "/web-template/launch/mavros_mouse.launch", 
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