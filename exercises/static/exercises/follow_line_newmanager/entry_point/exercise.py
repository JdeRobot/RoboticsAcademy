import os.path
from typing import Callable

from src.libs.applications.compatibility.exercise_wrapper import CompatibilityExerciseWrapper


class Exercise(CompatibilityExerciseWrapper):
    def __init__(self, circuit: str, update_callback: Callable):
        current_path = os.path.dirname(__file__)

        super(Exercise, self).__init__(exercise_command=f"{current_path}/../exercise.py 0.0.0.0",
                                       gui_command=f"{current_path}/../gui.py 0.0.0.0 {circuit}",
                                       update_callback=update_callback)
