import os
from typing import List, Any
import roslaunch

from src.manager.launcher.launcher_interface import ILauncher, LauncherException

import logging


class RosProcessListener(roslaunch.pmon.ProcessListener):
    def __init__(self, *args, **kwargs):
        self.callback = kwargs.get('callback', None)

    def process_died(self, name, exit_code):
        print(f"ROS process {name} terminated with code {exit_code}")
        if self.callback is not None:
            self.callback(name, exit_code)


class LauncherRosApi(ILauncher):
    exercise_id: str
    type: str
    module: str
    resource_folders: List[str]
    model_folders: List[str]
    plugin_folders: List[str]
    parameters: List[str]
    launch_file: str

    # holder for roslaunch process
    launch: Any = None
    listener: Any = None

    def run(self, callback: callable = None):
        logging.getLogger("roslaunch").setLevel(logging.CRITICAL)

        # expand variables in configuration paths
        self._set_environment()
        launch_file = os.path.expandvars(self.launch_file)

        self.listener = RosProcessListener(callback=callback)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file], process_listeners=[self.listener])
        self.launch.start()

        if not self.launch.pm.is_alive():
            raise LauncherException("Exception launching ROS")

    def is_running(self):
        return self.launch.pm.is_alive()

    def terminate(self):
        if self.is_running():
            self.launch.shutdown()

    def _set_environment(self):
        resource_folders = [os.path.expandvars(path) for path in self.resource_folders]
        model_folders = [os.path.expandvars(path) for path in self.model_folders]
        plugin_folders = [os.path.expandvars(path) for path in self.plugin_folders]

        os.environ["GAZEBO_RESOURCE_PATH"] = f"{os.environ.get('GAZEBO_RESOURCE_PATH', '')}:{':'.join(resource_folders)}"
        os.environ["GAZEBO_MODEL_PATH"] = f"{os.environ.get('GAZEBO_MODEL_PATH', '')}:{':'.join(model_folders)}"
        os.environ["GAZEBO_PLUGIN_PATH"] = f"{os.environ.get('GAZEBO_PLUGIN_PATH', '')}:{':'.join(plugin_folders)}"
