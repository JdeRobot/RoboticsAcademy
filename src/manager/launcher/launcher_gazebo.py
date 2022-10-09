import shutil
import subprocess

from src.manager.launcher.launcher_interface import ILauncher


class LauncherGazebo(ILauncher):
    def __init__(self, config):
        self.config = config
        self.gazebo_command_line = shutil.which('gazebo')

    def run(self):
        parameters = ' '.join(self.config.get("parameters", []))
        world_file = self.config.get("world_file", "")
        process = subprocess.Popen([self.gazebo_command_line, parameters, world_file])
        return process
