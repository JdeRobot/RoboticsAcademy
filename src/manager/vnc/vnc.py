import time
import threading
import subprocess

class DockerThread(threading.Thread):
    def __init__(self, cmd):
        threading.Thread.__init__(self)
        self.cmd = cmd

    def run(self):
        subprocess.Popen(self.cmd, shell=True, stdout=subprocess.PIPE, bufsize=1024, universal_newlines=True)

    def call(self):
        subprocess.call(self.cmd, shell=True, stdout=subprocess.PIPE, bufsize=1024, universal_newlines=True)

class Vnc():

    def __init__(self, display, internal_port, external_port):
        self.internal_port = internal_port
        self.display = display
        self.external_port = external_port
    
    def start_vnc(self):
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

    def start_console(self, width, height):
        # Write display config and start the console
        width = int(width) / 10; height = int(height) / 18
        console_cmd = f"export DISPLAY=:1;"
      
        console_cmd += f"xterm -geometry {int(width)}x{int(height)} -fa 'Monospace' -fs 10 -bg black -fg white"

        console_thread = DockerThread(console_cmd)
        console_thread.start()


    def start_gzserver(self, exercise):
        roslaunch_thread = DockerThread(exercise)
        roslaunch_thread.start()
        repeat = True
        while repeat:
            try:
                stats_output = str(subprocess.check_output(['gz', 'stats', '-p', '-d', '1'], timeout=5))
                if "real-time factor" in str(stats_output):
                    repeat = False
                else:
                    repeat = True
                    time.sleep(0.2)
            except:
                repeat = False

    def start_gzclient(self, exercise, width, height):
        # Configure browser screen width and height for gzclient
        gzclient_config_cmds = ["echo [geometry] > ~/.gazebo/gui.ini;",
                                "echo x=0 >> ~/.gazebo/gui.ini;",
                                "echo y=0 >> ~/.gazebo/gui.ini;",
                                f"echo width={width} >> ~/.gazebo/gui.ini;",
                                f"echo height={height} >> ~/.gazebo/gui.ini;"]

       
	    # Write display config and start gzclient
        gzclient_cmd = (f"export DISPLAY=:0;{exercise}" + "".join(gzclient_config_cmds) + "gzclient --verbose")
        gzclient_thread = DockerThread(gzclient_cmd)
        gzclient_thread.start()