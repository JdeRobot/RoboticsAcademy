import time
import socket
from src.manager.manager.docker_thread.docker_thread import DockerThread
import subprocess
from typing import List, Any
import os
from src.manager.libs.process_utils import wait_for_xserver

class Vnc_server:
    threads: List[Any] = []
    running: bool = False

    def start_vnc(self, display, internal_port, external_port):
        # Start X server in display
        xserver_cmd = f"/usr/bin/Xorg -quiet -noreset +extension GLX +extension RANDR +extension RENDER -logfile ./xdummy.log -config ./xorg.conf {display}"
        xserver_thread = DockerThread(xserver_cmd)
        xserver_thread.start()
        self.threads.append(xserver_thread)
        wait_for_xserver(display)

        # Start VNC server without password, forever running in background
        x11vnc_cmd = f"x11vnc -quiet -display {display} -nopw -forever -xkb -bg -rfbport {internal_port}"
        x11vnc_thread = DockerThread(x11vnc_cmd)
        x11vnc_thread.start()
        self.threads.append(x11vnc_thread)

        # Start noVNC with default port 6080 listening to VNC server on 5900
        if self.get_ros_version() == '2':
            novnc_cmd = f"/noVNC/utils/novnc_proxy --listen {external_port} --vnc localhost:{internal_port}"
        else:
            novnc_cmd = f"/noVNC/utils/launch.sh --listen {external_port} --vnc localhost:{internal_port}"

        novnc_thread = DockerThread(novnc_cmd)
        novnc_thread.start()
        self.threads.append(novnc_thread)
        self.running = True

        self.wait_for_port("localhost", internal_port)

    def start_vnc_gpu(self,display, internal_port, external_port, dri_path):
        # Start X and VNC servers
        turbovnc_cmd = f"export VGL_DISPLAY={dri_path} && export TVNC_WM=startlxde && /opt/TurboVNC/bin/vncserver {display} -geometry '1920x1080' -vgl -noreset -SecurityTypes None -rfbport {internal_port}"
        turbovnc_thread = DockerThread(turbovnc_cmd)
        turbovnc_thread.start()
        self.threads.append(turbovnc_thread)
        wait_for_xserver(display)

        # Start noVNC with default port 6080 listening to VNC server on 5900
        if self.get_ros_version() == '2':
            novnc_cmd = f"/noVNC/utils/novnc_proxy --listen {external_port} --vnc localhost:{internal_port}"
        else:
            novnc_cmd = f"/noVNC/utils/launch.sh --listen {external_port} --vnc localhost:{internal_port}"

        novnc_thread = DockerThread(novnc_cmd)
        novnc_thread.start()
        self.threads.append(novnc_thread)
        self.running = True

        self.wait_for_port("localhost", internal_port)
        self.wait_for_port("localhost", external_port)

        self.create_desktop_icon()
        self.create_gzclient_icon()


    def wait_for_port(self, host, port, timeout=20):
        start_time = time.time()
        while True:
            if time.time() - start_time > timeout:
                raise TimeoutError(f"Port {port} on {host} didn't become available within {timeout} seconds.")
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.settimeout(1)
                    sock.connect((host, port))
                break
            except (ConnectionRefusedError, TimeoutError):
                time.sleep(1)

    def is_running(self):
        return self.running


    def terminate(self):
        for thread in self.threads:
            thread.terminate()
            thread.join()
            self.running = False
        

    def get_ros_version(self):
        output = subprocess.check_output(['bash', '-c', 'echo $ROS_VERSION'])
        return output.decode('utf-8').strip()
    
    def create_desktop_icon(self):
        try:
            desktop_dir = os.path.expanduser('~/Desktop')
            if not os.path.exists(desktop_dir):
             os.makedirs(desktop_dir)
            desktop_path = os.path.join(desktop_dir, 'terminal_launcher.desktop')
            with open(desktop_path, 'w') as f:
                f.write("""[Desktop Entry]
                    Name=Open Terminal
                    Exec=xterm
                    Icon=utilities-terminal
                    Type=Application
                    Encoding=UTF-8
                    Terminal=false
                    Categories=None;""")
            os.chmod(desktop_path, 0o755)
        except Exception as err:
            print(err)

    def create_gzclient_icon(self):
        desktop_dir = os.path.expanduser('~/Desktop')
        if not os.path.exists(desktop_dir):
            os.makedirs(desktop_dir)
        desktop_path = os.path.join(desktop_dir, 'gzclient_launcher.desktop')

        try:
            with open(desktop_path, 'w') as f:
                f.write("""[Desktop Entry]
    Name=Gazebo Client
    Exec=gzclient
    Icon=gazebo
    Type=Application
    Encoding=UTF-8
    Terminal=false
    Categories=None;""")
            os.chmod(desktop_path, 0o755)
            print("Icono de gzclient creado con éxito en el escritorio.")
        except Exception as e:
            print(f"Error al crear el icono de gzclient: {e}")
