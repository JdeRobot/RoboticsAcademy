import os
from src.manager.manager.launcher.launcher_interface import ILauncher
from src.manager.manager.docker_thread.docker_thread import DockerThread
from src.manager.libs.process_utils import wait_for_xserver
from typing import List, Any


class LauncherDronesRos2(ILauncher):
    type: str
    module: str
    launch_file: str
    threads: List[Any] = []

    def run(self, callback):
        # Start X server in display
        xserver_cmd = f"/usr/bin/Xorg -quiet -noreset +extension GLX +extension RANDR +extension RENDER -logfile ./xdummy.log -config ./xorg.conf :0"
        xserver_thread = DockerThread(xserver_cmd)
        xserver_thread.start()
        wait_for_xserver(":0")
        self.threads.append(xserver_thread)

        # expand variables in configuration paths
        world_file = os.path.expandvars(self.launch_file)

        # Launching MicroXRCE and Aerostack2 nodes
        as2_launch_cmd = f"ros2 launch jderobot_drones as2_default_classic_gazebo.launch.py world_file:={world_file}"

        as2_launch_thread = DockerThread(as2_launch_cmd)
        as2_launch_thread.start()
        self.threads.append(as2_launch_thread)

        # Launching gzserver and PX4
        px4_launch_cmd = f"$AS2_GZ_ASSETS_SCRIPT_PATH/default_run.sh {world_file}"

        px4_launch_thread = DockerThread(px4_launch_cmd)
        px4_launch_thread.start()
        self.threads.append(px4_launch_thread)

        

    def is_running(self):
        return True

    def terminate(self):
        if self.is_running():
            for thread in self.threads:
                thread.terminate()
                thread.join()
           