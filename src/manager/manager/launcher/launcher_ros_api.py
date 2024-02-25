import os
import time
from typing import List, Any
from src.manager.manager.docker_thread.docker_thread import DockerThread
from src.manager.libs.process_utils import wait_for_xserver
from src.manager.libs.process_utils import wait_for_process_to_start
import roslaunch
import rospy


from src.manager.manager.launcher.launcher_interface import ILauncher, LauncherException

import logging


class RosProcessListener(roslaunch.pmon.ProcessListener):
    def __init__(self, *args, **kwargs):
        self.callback = kwargs.get('callback', None)

    def process_died(self, name, exit_code):
        print(f"ROS process {name} terminated with code {exit_code}")
        if self.callback is not None:
            self.callback(name, exit_code)


class LauncherRosApi(ILauncher):
    type: str
    module: str
    launch_file: str
    threads: List[Any] = []

    # holder for roslaunch process
    launch: Any = None
    listener: Any = None

    def run(self, callback: callable = None):
        logging.getLogger("roslaunch").setLevel(logging.CRITICAL)

        # Start X server in display
        xserver_cmd = f"/usr/bin/Xorg -quiet -noreset +extension GLX +extension RANDR +extension RENDER -logfile ./xdummy.log -config ./xorg.conf :0"
        xserver_thread = DockerThread(xserver_cmd)
        xserver_thread.start()
        wait_for_xserver(":0")
        self.threads.append(xserver_thread)

        self.listener = RosProcessListener(callback=callback)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(
            uuid, [self.launch_file], process_listeners=[self.listener])
        self.launch.start()

        wait_for_process_to_start("rosmaster", timeout=60)
        wait_for_process_to_start("gzserver", timeout=60)

        if not self.launch.pm.is_alive():
            raise LauncherException("Exception launching ROS")

    def is_running(self):
        return self.launch.pm.is_alive()

    def wait_for_shutdown(self, timeout=30):
        print("Waiting for ROS and Gazebo to shutdown")
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and self.is_running():
            if rospy.Time.now().to_sec() - start_time > timeout:
                print("Timeout while waiting for ROS and Gazebo to shutdown")
                break
            rospy.sleep(0.5)

    def terminate(self):
        try:
            for thread in self.threads:
                thread.terminate()
                thread.join()
            self.launch.shutdown()
            self.wait_for_shutdown()
        except Exception as e:
            print("Exception shutting down ROS")
