from src.manager.launcher.launcher_interface import ILauncher, LauncherException
from src.manager.vnc.docker_thread import DockerThread


class LauncherConsole(ILauncher):
    display: str
    internal_port: str
    external_port: str
    height: int
    width: int

    def run(self, callback):
        xserver_cmd = f"/usr/bin/Xorg -noreset +extension GLX +extension RANDR +extension RENDER -logfile ./xdummy.log -config ./xorg.conf {self.display}"
        xserver_thread = DockerThread(xserver_cmd)
        xserver_thread.start()
        # Start VNC server without password, forever running in background
        x11vnc_cmd = f"x11vnc -display {self.display} -nopw -forever -xkb -bg -rfbport {self.internal_port}"
        x11vnc_thread = DockerThread(x11vnc_cmd)
        x11vnc_thread.start()

         # Start noVNC with default port 6080 listening to VNC server on 5900
        novnc_cmd = f"/noVNC/utils/launch.sh --listen {self.external_port} --vnc localhost:{self.internal_port}"
        novnc_thread = DockerThread(novnc_cmd)
        novnc_thread.start()

        # Write display config and start the console
        width = int(self.width) / 10; height = int(self.height) / 18
        console_cmd = f"export DISPLAY=:1;"
        print('test starting console')
        console_cmd += f"xterm -geometry {int(self.width)}x{int(self.height)} -fa 'Monospace' -fs 10 -bg black -fg white"

        console_thread = DockerThread(console_cmd)
        console_thread.start()

    def is_running():
        pass

    def terminate():
        pass

    def died():
        pass