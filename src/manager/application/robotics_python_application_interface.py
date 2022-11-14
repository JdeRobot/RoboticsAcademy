class IRoboticsPythonApplication:
    def load_code(self, code: str):
        raise NotImplementedError("Exercise brains must implement load_code")

    def run(self):
        raise NotImplementedError("Exercise brains must implement run")

    def stop(self):
        raise NotImplementedError("Exercise brains must implement stop")

    def restart(self):
        raise NotImplementedError("Exercise brains must implement restart")

    def terminate(self):
        raise NotImplementedError("Exercise brains must implement terminate")

    @property
    def is_alive(self):
        raise NotImplementedError("Exercise brains must implement restart")