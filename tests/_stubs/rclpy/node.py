class Node:
    def __init__(self, name=None):
        self.name = name

    def create_subscription(self, *args, **kwargs):
        return object()

    def create_publisher(self, *args, **kwargs):
        return object()

    def get_logger(self):
        class Logger:
            def info(self, *a, **k):
                pass

            def debug(self, *a, **k):
                pass

        return Logger()
