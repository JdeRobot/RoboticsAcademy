from src.manager.application.robotics_python_application_interface import IRoboticsPythonApplication


class RoboticsApplication(IRoboticsPythonApplication):
    def terminate(self):
        pass

    def load_code(self, code: str):
        pass

    def run(self):
        pass

    def stop(self):
        pass

    def restart(self):
        pass

    def pause(self):
        pass

    @property
    def is_alive(self):
        pass