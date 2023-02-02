from vnc.vnc_server import VncServer
from vnc.docker_thread import DockerThread
import subprocess
import time

class GzbView(VncServer):
    def __init__(self, display, internal_port, external_port):
        super().start_vnc(display, internal_port, external_port)

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
        gzclient_config_cmds = f"echo [geometry] > ~/.gazebo/gui.ini; echo x=0 >> ~/.gazebo/gui.ini; echo y=0 >> ~/.gazebo/gui.ini; echo width={width} >> ~/.gazebo/gui.ini; echo height={height} >> ~/.gazebo/gui.ini;"

	    # Write display config and start gzclient
        gzclient_cmd = (f"export DISPLAY=:0; {exercise}; {gzclient_config_cmds} gzclient --verbose")
        gzclient_thread = DockerThread(gzclient_cmd)
        gzclient_thread.start() 