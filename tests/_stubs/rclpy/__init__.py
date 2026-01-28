from .node import Node
from .qos import ReliabilityPolicy, QoSProfile, HistoryPolicy

_inited = False


def init():
    global _inited
    _inited = True


def ok():
    return _inited


def shutdown():
    global _inited
    _inited = False
