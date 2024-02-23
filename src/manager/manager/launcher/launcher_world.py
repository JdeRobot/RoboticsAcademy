from typing import Optional
from pydantic import BaseModel

from src.manager.libs.process_utils import get_class, class_from_module, get_ros_version
from src.manager.ram_logging.log_manager import LogManager
from src.manager.manager.launcher.launcher_interface import ILauncher

worlds = {
    "gazebo":
    {"1": [{
        "type": "module",
        "module": "ros_api",
        "parameters": [],
        "launch_file": [],
    }], "2": [{
        "type": "module",
        "module": "ros2_api",
        "parameters": [],
        "launch_file": [],
    }]},
    "drones":
    {"1": [{
        "type": "module",
        "module": "drones",
        "resource_folders": [],
        "model_folders": [],
        "plugin_folders": [],
        "parameters": [],
        "launch_file": [],
    }], "2": [{
        "type": "module",
        "module": "drones_ros2",
        "resource_folders": [],
        "model_folders": [],
        "plugin_folders": [],
        "parameters": [],
        "launch_file": [],
    }]},
    "physical": {}
}


class LauncherWorld(BaseModel):
    world: str
    launch_file_path: str
    module: str = '.'.join(__name__.split('.')[:-1])
    ros_version: int = get_ros_version()
    launchers: Optional[ILauncher] = []

    def run(self):
        for module in worlds[self.world][str(self.ros_version)]:
            module["launch_file"] = self.launch_file_path
            launcher = self.launch_module(module)
            self.launchers.append(launcher)

    def terminate(self):
        LogManager.logger.info("Terminating world launcher")
        if self.launchers:
            for launcher in self.launchers:
                launcher.terminate()
        self.launchers = []

    def launch_module(self, configuration):
        def process_terminated(name, exit_code):
            LogManager.logger.info(
                f"LauncherEngine: {name} exited with code {exit_code}")
            if self.terminated_callback is not None:
                self.terminated_callback(name, exit_code)

        launcher_module_name = configuration["module"]
        launcher_module = f"{self.module}.launcher_{launcher_module_name}.Launcher{class_from_module(launcher_module_name)}"
        launcher_class = get_class(launcher_module)
        launcher = launcher_class.from_config(launcher_class, configuration)
        launcher.run(process_terminated)
        return launcher

    def launch_command(self, configuration):
        pass


class LauncherWorldException(Exception):
    def __init__(self, message):
        super(LauncherWorldException, self).__init__(message)
