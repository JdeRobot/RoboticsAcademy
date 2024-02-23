import os
from typing import List, Any
import time
import stat

from src.manager.manager.launcher.launcher_interface import ILauncher, LauncherException
from src.manager.manager.docker_thread.docker_thread import DockerThread
import subprocess

import logging

class LauncherRos2Api(ILauncher):
    type: str
    module: str
    launch_file: str
    threads: List[Any] = []

    def run(self,callback):
        DRI_PATH = os.path.join("/dev/dri", os.environ.get("DRI_NAME", "card0"))
        ACCELERATION_ENABLED = self.check_device(DRI_PATH)

        logging.getLogger("roslaunch").setLevel(logging.CRITICAL)

        xserver_cmd = f"/usr/bin/Xorg -quiet -noreset +extension GLX +extension RANDR +extension RENDER -logfile ./xdummy.log -config ./xorg.conf :0"
        xserver_thread = DockerThread(xserver_cmd)
        xserver_thread.start()
        self.threads.append(xserver_thread)

        if (ACCELERATION_ENABLED):
            exercise_launch_cmd = f"export VGL_DISPLAY={DRI_PATH}; vglrun ros2 launch {self.launch_file}"
        else:
            exercise_launch_cmd = f"ros2 launch {self.launch_file}"

        exercise_launch_thread = DockerThread(exercise_launch_cmd)
        exercise_launch_thread.start()
    
    def check_device(self, device_path):
        try:
            return stat.S_ISCHR(os.lstat(device_path)[stat.ST_MODE])
        except:
            return False

    def terminate(self):
        if self.threads is not None:
            for thread in self.threads:
                thread.terminate()
                thread.join()
            
        kill_cmd = 'pkill -9 -f '
        cmd = kill_cmd + 'gzserver'
        subprocess.call(cmd, shell=True, stdout=subprocess.PIPE, bufsize=1024, universal_newlines=True)
        cmd = kill_cmd + 'spawn_model.launch.py'
        subprocess.call(cmd, shell=True, stdout=subprocess.PIPE, bufsize=1024, universal_newlines=True)

        kill_cmd = 'pkill -9 -f '
        cmd = kill_cmd + 'gzserver'
        subprocess.call(cmd, shell=True, stdout=subprocess.PIPE, bufsize=1024, universal_newlines=True)
        cmd = kill_cmd + 'spawn_model.launch.py'
        subprocess.call(cmd, shell=True, stdout=subprocess.PIPE, bufsize=1024, universal_newlines=True)
