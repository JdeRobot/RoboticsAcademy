from pathlib import Path

PATH = Path(
    "/usr/local/lib/python3.10/dist-packages/robotics_application_manager/manager/manager.py"
)
text = PATH.read_text(encoding="utf-8")

old = """        fds = os.listdir("/dev/pts/")
        console_fd = str(max(map(int, fds[:-1])))

        self.unpause_sim()
        self.application_process = subprocess.Popen(
            ["python3", entrypoint],
            stdin=open("/dev/pts/" + console_fd, "r"),
            stdout=open("/dev/pts/" + console_fd, "w"),
            stderr=sys.stdout,
            bufsize=1024,
            universal_newlines=True,
        )
"""

new = """        fds = [fd for fd in os.listdir("/dev/pts/") if fd.isdigit()]

        self.unpause_sim()
        if fds:
            console_fd = str(max(map(int, fds)))
            self.application_process = subprocess.Popen(
                ["python3", entrypoint],
                stdin=open("/dev/pts/" + console_fd, "r"),
                stdout=open("/dev/pts/" + console_fd, "w"),
                stderr=sys.stdout,
                bufsize=1024,
                universal_newlines=True,
            )
        else:
            # Some headless launches expose only /dev/pts/ptmx; avoid crashing on max([]).
            self.application_process = subprocess.Popen(
                ["python3", entrypoint],
                stdin=subprocess.DEVNULL,
                stdout=sys.stdout,
                stderr=sys.stdout,
                bufsize=1024,
                universal_newlines=True,
            )
"""

if old in text:
    text = text.replace(old, new, 1)
    PATH.write_text(text, encoding="utf-8")
    print("patched:", PATH)
else:
    print("pattern not found, skipped:", PATH)
