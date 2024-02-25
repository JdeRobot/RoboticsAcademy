import os
import subprocess
from src.manager.manager.launcher.launcher_interface import ILauncher, LauncherException
from src.manager.manager.docker_thread.docker_thread import DockerThread
from src.manager.libs.process_utils import wait_for_xserver
from typing import List, Any
import psutil
import threading

class LauncherDrones(ILauncher):
    exercise_id: str
    type: str
    module: str
    parameters: List[str]
    launch_file: str
    process: Any = None
    threads: List[Any] = []

    # holder for roslaunch process
    launch: Any = None

    def run(self, callback: callable = None):
        # Start X server in display
        xserver_cmd = f"/usr/bin/Xorg -quiet -noreset +extension GLX +extension RANDR +extension RENDER -logfile ./xdummy.log -config ./xorg.conf :0"
        xserver_thread = DockerThread(xserver_cmd)
        xserver_thread.start()
        wait_for_xserver(":0")
        self.threads.append(xserver_thread)
        self.launch_file = os.path.expandvars(self.launch_file)

        # Inicia el proceso en un hilo separado
        self.launch = DockerThread(f'python3 {self.launch_file}' )
        self.launch.start()
        self.threads.append(self.launch)

    def is_running(self):
        return True

    def terminate(self):
        try:
            for thread in self.threads:
                thread.terminate()
                thread.join()
        except Exception as e:
            print("Exception shutting down ROS")

    def died(self):
        pass
