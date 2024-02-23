from pydantic import BaseModel


class ILauncher(BaseModel):
    def run(self, callback: callable):
        raise NotImplemented("Launcher must implement run method")

    def is_running(self):
        raise NotImplemented("Launcher must implement run method")

    def terminate(self):
        raise NotImplemented("Launcher must implement run method")

    def died(self, callback):
        raise NotImplemented("Launcher must implement run method")

    def from_config(cls, config):
        obj = cls(**config)
        return obj


class LauncherException(Exception):
    def __init__(self, message):
        super(LauncherException, self).__init__(message)