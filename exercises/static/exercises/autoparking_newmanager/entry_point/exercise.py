import os.path
from typing import Callable, Any

from src.manager.libs.applications.compatibility.exercise_wrapper import CompatibilityExerciseWrapper


class Exercise(CompatibilityExerciseWrapper):
    def __init__(self,  gui_server: Any, update_callback: Callable):
        current_path = os.path.dirname(__file__)

        super(Exercise, self).__init__(exercise_command=f"{current_path}/../python_template/ros1_noetic/exercise.py",
                                       update_callback=update_callback,
                                       gui_server=gui_server
                                       )
