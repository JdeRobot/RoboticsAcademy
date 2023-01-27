from vnc.vnc_server import Vnc_server
from vnc.docker_thread import DockerThread

class Console_view(Vnc_server):
    def __init__(self, display, internal_port, external_port):
        super().start_vnc(display, internal_port, external_port)
    
    def start_console(self, width, height):
        # Write display config and start the console
        width = int(width) / 10; height = int(height) / 18
        console_cmd = f"export DISPLAY=:1;"
        print('test starting console')
        console_cmd += f"xterm -geometry {int(width)}x{int(height)} -fa 'Monospace' -fs 10 -bg black -fg white"

        console_thread = DockerThread(console_cmd)
        console_thread.start()
        
