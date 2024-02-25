from src.manager.manager.launcher.launcher_interface import ILauncher
from src.manager.manager.docker_thread.docker_thread import DockerThread
from src.manager.manager.vnc.vnc_server import Vnc_server
import time
import os
import stat


class LauncherRobotDisplayView(ILauncher):
    display: str
    internal_port: str
    external_port: str
    height: int
    width: int
    running = False
    threads = []

    def run(self, callback):
        DRI_PATH = os.path.join("/dev/dri", os.environ.get("DRI_NAME", "card0"))
        ACCELERATION_ENABLED = self.check_device(DRI_PATH)

        robot_display_vnc = Vnc_server()
        
        if (ACCELERATION_ENABLED):
            robot_display_vnc.start_vnc_gpu(self.display, self.internal_port, self.external_port,DRI_PATH)
            # Write display config and start the console
            console_cmd = f"export VGL_DISPLAY={DRI_PATH}; export DISPLAY={self.display}; /usr/bin/Xorg -noreset +extension GLX +extension RANDR +extension RENDER -logfile ./xdummy.log -config ./xorg.conf {self.display}"
        else:
            robot_display_vnc.start_vnc(self.display, self.internal_port, self.external_port)
            # Write display config and start the console
            console_cmd = f"export DISPLAY={self.display};/usr/bin/Xorg -noreset +extension GLX +extension RANDR +extension RENDER -logfile ./xdummy.log -config ./xorg.conf {self.display}"

        console_thread = DockerThread(console_cmd)
        console_thread.start()
        self.threads.append(console_thread)

        self.running = True        

    def check_device(self, device_path):
        try:
            return stat.S_ISCHR(os.lstat(device_path)[stat.ST_MODE])
        except:
            return False

    def is_running(self):
        return self.running

    def terminate(self):
        for thread in self.threads:
            thread.terminate()
            thread.join()
        self.running = False

    def died(self):
        pass
