import os.path

from src.libs.applications.compatibility_exercise_wrapper import CompatibilityExerciseWrapper


class Exercise(CompatibilityExerciseWrapper):
    def __init__(self, circuit: str):
        current_path = os.path.dirname(__file__)

        super(Exercise, self).__init__(f"{current_path}/../exercise.py 0.0.0.0",
                                       f"{current_path}/../gui.py 0.0.0.0 {circuit}")
