from vnc.docker_thread import DockerThread



class VncServer:
    def start_vnc(self,display, internal_port, external_port):
        xserver_cmd = f"/usr/bin/Xorg -noreset +extension GLX +extension RANDR +extension RENDER -logfile ./xdummy.log -config ./xorg.conf {display}"
        xserver_thread = DockerThread(xserver_cmd)
        xserver_thread.start()
        # Start VNC server without password, forever running in background
        x11vnc_cmd = f"x11vnc -display {display} -nopw -forever -xkb -bg -rfbport {internal_port}"
        x11vnc_thread = DockerThread(x11vnc_cmd)
        x11vnc_thread.start()

         # Start noVNC with default port 6080 listening to VNC server on 5900
        novnc_cmd = f"/noVNC/utils/launch.sh --listen {external_port} --vnc localhost:{internal_port}"
        novnc_thread = DockerThread(novnc_cmd)
        novnc_thread.start()



