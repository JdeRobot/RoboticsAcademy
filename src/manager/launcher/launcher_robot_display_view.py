from src.manager.launcher.launcher_interface import ILauncher
from src.manager.docker_thread.docker_thread import DockerThread
import time


class LauncherRobotDisplayView(ILauncher):
    display: str
    internal_port: str
    external_port: str
    height: int
    width: int
    running = False

    def run(self, callback):
        xserver_cmd = f"/usr/bin/Xorg -noreset +extension GLX +extension RANDR +extension RENDER -logfile ./xdummy.log -config ./xorg.conf {self.display}"
        xserver_thread = DockerThread(xserver_cmd)
        xserver_thread.start()
        time.sleep(0.1)
        # Start VNC server without password, forever running in background
        x11vnc_cmd = f"x11vnc -display {self.display} -nopw -forever -xkb -bg -rfbport {self.internal_port}"
        x11vnc_thread = DockerThread(x11vnc_cmd)
        x11vnc_thread.start()

        # Start noVNC with default port 2303 listening to VNC server on 5902
        novnc_cmd = f"/noVNC/utils/launch.sh --listen {self.external_port} --vnc localhost:{self.internal_port}"
        novnc_thread = DockerThread(novnc_cmd)
        novnc_thread.start()

        # Write display config and start RobotDisplayViewVNC
        robot_cmd = (f"export DISPLAY=2;")
        robot_thread = DockerThread(robot_cmd)

        robot_thread.start()
        self.running = True

    def is_running(self):
        return self.running

    def terminate(self):
        pass

    def died(self):
        pass
