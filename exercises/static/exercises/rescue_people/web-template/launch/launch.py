#!/usr/bin/env python3

import os
import stat
import rospy
import subprocess


# Function to check if a device exists
def check_device(device_path):
    try:
        return stat.S_ISCHR(os.lstat(device_path)[stat.ST_MODE])
    except:
        return False

DRI_PATH = "/dev/dri/card0"
ACCELERATION_ENABLED = check_device(DRI_PATH)


class Tests():
    def test_px4(self):
        while True:
            args = ['./Firmware/build/px4_sitl_default/bin/px4-commander', 'check']
            process = subprocess.Popen(args, stdout=subprocess.PIPE)
            output, err = process.communicate()
            idx = output.find('Prearm check: ')
            if output[idx+14:idx+16] == 'OK':
                break
            else:
                rospy.sleep(2)

    def test_mavros(self, ns=''):
        rospy.wait_for_service(ns + '/mavros/cmd/arming', 30)

    def test_gazebo(self):
        rospy.wait_for_service('/gazebo/spawn_sdf_model', 30)


class Launch(Tests):
    def __init__(self):
        env = {'GAZEBO_MODEL_PATH': '$GAZEBO_MODEL_PATH:/Firmware/Tools/sitl_gazebo/models:/opt/ros/melodic/share/drone_assets/models:/opt/ros/melodic/share/drone_assets/urdf:/drones/drone_assets/models'}
        os.environ.update(env)

        args = ['/opt/ros/melodic/bin/roscore']
        subprocess.Popen(args)

        rospy.init_node('launch', anonymous=True)

    def main(self):
        try:
            args = ['/opt/ros/melodic/bin/roslaunch', '/RoboticsAcademy/exercises/rescue_people/web-template/launch/gazebo.launch', '--wait']
            if ACCELERATION_ENABLED: args.insert(0, 'vglrun')
            subprocess.Popen(args, stdout=subprocess.PIPE)

            rospy.logwarn('[ GAZEBO ] Waiting for spawn_sdf_model service')
            self.test_gazebo()

            args = ['/opt/ros/melodic/bin/roslaunch', '/RoboticsAcademy/exercises/rescue_people/web-template/launch/px4.launch']
            if ACCELERATION_ENABLED: args.insert(0, 'vglrun')
            subprocess.Popen(args, stdout=subprocess.PIPE)

            rospy.logwarn('[ PX4-SITL ] Waiting for Pre-Arm checks')
            self.test_px4()

            args = ['/opt/ros/melodic/bin/roslaunch', '/RoboticsAcademy/exercises/rescue_people/web-template/launch/mavros.launch']
            if ACCELERATION_ENABLED: args.insert(0, 'vglrun')
            subprocess.Popen(args, stdout=subprocess.PIPE)

            rospy.logwarn('[ MAVROS ] Waiting for mavros')
            self.test_mavros()

        except Exception as e:
            print('[ ERROR ]', e)


if __name__ == '__main__':
    launch = Launch()
    launch.main()
