"""Launcher for Console"""""
import time
from src.manager.launcher.launcher_interface import ILauncher
from src.manager.docker_thread.docker_thread import DockerThread



class LauncherConsole(ILauncher):
    """Launcher for Console"""
    display: str
    internal_port: str
    external_port: str
    running = False
    threads = []

    def run(self, callback):
        xserver_cmd = f"/usr/bin/Xorg -noreset +extension GLX +extension RANDR +extension RENDER -logfile ./xdummy.log -config ./xorg.conf {self.display}"
        xserver_thread = DockerThread(xserver_cmd)
        xserver_thread.start()
        self.threads.append(xserver_thread)
        time.sleep(0.1)
        # Start VNC server without password, forever running in background
        x11vnc_cmd = f"x11vnc -display {self.display} -nopw -forever -xkb -bg -rfbport {self.internal_port}"
        x11vnc_thread = DockerThread(x11vnc_cmd)
        x11vnc_thread.start()
        self.threads.append(x11vnc_thread)

        # Start noVNC with default port 6080 listening to VNC server on 5900
        novnc_cmd = f"/noVNC/utils/launch.sh --listen {self.external_port} --vnc localhost:{self.internal_port}"
        novnc_thread = DockerThread(novnc_cmd)
        novnc_thread.start()
        self.threads.append(novnc_thread)

        # Write display config and start the console
        console_cmd = "export DISPLAY=:1;xterm -geometry 100x10+0+0 -fa 'Monospace' -fs 10 -bg black -fg white"
        console_thread = DockerThread(console_cmd)
        console_thread.start()
        self.threads.append(console_thread)

        self.running = True

    def is_running(self):
        return self.running

    def terminate(self):
        for thread in self.threads:
            thread.terminate()
            thread.join()
        self.running = False

    def died(self):
        pass
