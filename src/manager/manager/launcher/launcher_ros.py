import os
import shutil
import subprocess
import traceback
from typing import Any, List

from src.manager.launcher.launcher_interface import ILauncher, LauncherException


class LauncherRos(ILauncher):
    """
    Launcher for ROS/Gazebo

    It's configuration should follow this spec:

    {
      "type": "module",
      "module": "ros",
      "resource_folders": [
        "$EXERCISE_FOLDER/launch"
      ],
      "model_folders": [
        "$CUSTOM_ROBOTS/f1/models"
      ],
      "plugin_folders": [
      ],
      "parameters": [],
      "launch_file": "$EXERCISE_FOLDER/launch/simple_line_follower_ros_headless_default.launch"
    }
    """
    exercise_id: str
    type: str
    module: str
    resource_folders: List[str]
    model_folders: List[str]
    plugin_folders: List[str]
    parameters: List[str]
    launch_file: str

    ros_command_line: str = shutil.which('roslaunch')
    process: Any = None

    def run(self):
        try:
            # generate entry_point environment variable
            os.environ["EXERCISE_FOLDER"] = f"{os.environ.get('EXERCISES_STATIC_FOLDER')}/{self.exercise_id}"

            # expand variables in configuration paths
            resource_folders = [os.path.expandvars(
                path) for path in self.resource_folders]
            model_folders = [os.path.expandvars(
                path) for path in self.model_folders]
            plugin_folders = [os.path.expandvars(
                path) for path in self.plugin_folders]
            launch_file = os.path.expandvars(self.launch_file)

            env = dict(os.environ)
            env["GAZEBO_RESOURCE_PATH"] = f"{env.get('GAZEBO_RESOURCE_PATH', '')}:{':'.join(resource_folders)}"
            env["GAZEBO_MODEL_PATH"] = f"{env.get('GAZEBO_MODEL_PATH', '')}:{':'.join(model_folders)}"
            env["GAZEBO_PLUGIN_PATH"] = f"{env.get('GAZEBO_PLUGIN_PATH', '')}:{':'.join(plugin_folders)}"

            parameters = " ".join(self.parameters)
            command = f"{self.ros_command_line} {parameters} {launch_file}"
            self.process = subprocess.Popen(command, env=env, shell=True,
                                            # stdin=subprocess.PIPE,
                                            # stdout=subprocess.PIPE,
                                            # stderr=subprocess.STDOUT
                                            )
            # print(self.process.communicate())
        except Exception as ex:
            traceback.print_exc()
            raise ex

    def is_running(self):
        return True if self.process.poll() is None else False

    def terminate(self):
        if self.is_running():
            self.process.terminate()
        else:
            raise LauncherException("The process is not running")
