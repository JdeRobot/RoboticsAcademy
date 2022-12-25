# Thanks to https://stackoverflow.com/questions/452969/does-python-have-an-equivalent-to-java-class-forname
# This should be moved to a utils library
import importlib.util
import os.path
import sys
from subprocess import Popen

import psutil


def get_class(kls):
    parts = kls.split('.')
    module = ".".join(parts[:-1])
    m = __import__(module)
    for comp in parts[1:]:
        m = getattr(m, comp)
    return m


def get_class_from_file(file_path: str, class_name: str):
    spec = importlib.util.spec_from_file_location("application", file_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[f"{class_name}"] = module
    spec.loader.exec_module(module)
    return getattr(module, class_name)


def class_from_module(module: str):
    """
    Capitalizes a module name to create class name
    """
    return ''.join([s.capitalize() for s in module.split('_')])


def stop_process_and_children(process: Popen, signal: int = 9, timeout: int = None):
    """
    Stops a list of processes waiting for them to stop
    """
    # collect processes to stop
    proc = psutil.Process(process.pid)
    children = proc.children(recursive=True)
    children.append(proc)

    # send signal to processes
    for p in children:
        try:
            p.send_signal(signal)
        except psutil.NoSuchProcess:
            pass

    gone, alive = psutil.wait_procs(children, timeout=timeout)
    return gone, alive


class classproperty(property):
    def __get__(self, cls, owner):
        return classmethod(self.fget).__get__(None, owner)()

def singleton(cls):
    instances = {}
    def get_instance():
        if cls not in instances:
            instances[cls] = cls()
        return instances[cls]
    return get_instance()