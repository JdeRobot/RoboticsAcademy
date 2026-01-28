class ReliabilityPolicy:
    BEST_EFFORT = 1
    RELIABLE = 2


class HistoryPolicy:
    KEEP_LAST = 1
    KEEP_ALL = 2


class QoSProfile:
    def __init__(self, reliability=None, history=None, depth=None):
        self.reliability = reliability
        self.history = history
        self.depth = depth
