from src.manager.manager.launcher.launcher_interface import ILauncher
from src.manager.manager.docker_thread.docker_thread import DockerThread
from src.manager.manager.vnc.vnc_server import Vnc_server
from src.manager.libs.process_utils import wait_for_process_to_start, check_gpu_acceleration
import subprocess
import time
import os
import stat
from typing import List, Any


class LauncherGazeboView(ILauncher):
    display: str
    internal_port: int
    external_port: int
    height: int
    width: int
    running: bool = False
    threads: List[Any] = []
    gz_vnc: Any = Vnc_server()

    def run(self, callback):
        DRI_PATH = self.get_dri_path()
        ACCELERATION_ENABLED = self.check_device(DRI_PATH)

        # Configure browser screen width and height for gzclient
        gzclient_config_cmds = f"echo [geometry] > ~/.gazebo/gui.ini; echo x=0 >> ~/.gazebo/gui.ini; echo y=0 >> ~/.gazebo/gui.ini; echo width={self.width} >> ~/.gazebo/gui.ini; echo height={self.height} >> ~/.gazebo/gui.ini;"

        if ACCELERATION_ENABLED:
            # Starts xserver, x11vnc and novnc
            self.gz_vnc.start_vnc_gpu(
                self.display, self.internal_port, self.external_port, DRI_PATH)
            # Write display config and start gzclient
            gzclient_cmd = (
                f"export DISPLAY={self.display}; {gzclient_config_cmds} export VGL_DISPLAY={DRI_PATH}; vglrun gzclient --verbose")
        else:
            # Starts xserver, x11vnc and novnc
            self.gz_vnc.start_vnc(
                self.display, self.internal_port, self.external_port)
            # Write display config and start gzclient
            gzclient_cmd = (
                f"export DISPLAY={self.display}; {gzclient_config_cmds} gzclient --verbose")

        gzclient_thread = DockerThread(gzclient_cmd)
        gzclient_thread.start()
        self.threads.append(gzclient_thread)

        process_name = "gzclient"
        wait_for_process_to_start(process_name, timeout=60)

        self.running = True

    def check_device(self, device_path):
        try:
            return stat.S_ISCHR(os.lstat(device_path)[stat.ST_MODE])
        except:
            return False

    def is_running(self):
        return self.running

    def terminate(self):
        self.gz_vnc.terminate()
        for thread in self.threads:
            thread.terminate()
            thread.join()
        self.running = False

    def died(self):
        pass

    def get_dri_path(self):
        directory_path = '/dev/dri'
        dri_path = ""
        if os.path.exists(directory_path) and os.path.isdir(directory_path):
            files = os.listdir(directory_path)
            if ("card1" in files):
                dri_path = os.path.join(
                    "/dev/dri", os.environ.get("DRI_NAME", "card1"))
            else:
                dri_path = os.path.join(
                    "/dev/dri", os.environ.get("DRI_NAME", "card0"))
        return dri_path
