from src.manager.manager.launcher.launcher_interface import ILauncher
from src.manager.manager.docker_thread.docker_thread import DockerThread
from src.manager.manager.vnc.vnc_server import Vnc_server
from src.manager.libs.process_utils import  check_gpu_acceleration
import os
import stat
from typing import List, Any


class LauncherConsole(ILauncher):
    display: str
    internal_port: int
    external_port: int
    running:bool = False
    threads: List[Any] = []
    console_vnc: Any = Vnc_server()

    def run(self, callback):
        DRI_PATH = os.path.join("/dev/dri", os.environ.get("DRI_NAME", "card0"))
        ACCELERATION_ENABLED = False

        
        
        if (ACCELERATION_ENABLED):
            self.console_vnc.start_vnc_gpu(self.display, self.internal_port, self.external_port,DRI_PATH)
            # Write display config and start the console
            console_cmd = f"export VGL_DISPLAY={DRI_PATH}; export DISPLAY={self.display}; vglrun xterm -fullscreen -sb -fa 'Monospace' -fs 10 -bg black -fg white"
        else:
            self.console_vnc.start_vnc(self.display, self.internal_port, self.external_port)
            # Write display config and start the console
            console_cmd = f"export DISPLAY={self.display};xterm -geometry 100x10+0+0 -fa 'Monospace' -fs 10 -bg black -fg white"

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
        self.console_vnc.terminate()
        for thread in self.threads:
            thread.terminate()
            thread.join()
        self.running = False

    def died(self):
        pass