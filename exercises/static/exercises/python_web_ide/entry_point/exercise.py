import os.path
from typing import Callable

from src.libs.applications.compatibility.robotics_application_wrapper import RoboticsApplicationWrapper


class Exercise(RoboticsApplicationWrapper):
    def __init__(self, circuit: str, update_callback: Callable):

        super(Exercise, self).__init__(update_callback=update_callback)