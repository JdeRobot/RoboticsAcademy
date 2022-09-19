from transitions import Machine


class ExerciseManager:
    states = [
        "idle",
        "connected",
        "ready",
        "running",
        "paused"
    ]

    transitions = [
        {'trigger': 'connect', 'source': 'idle', 'dest': 'connected'},
        {'trigger': 'launch', 'source': 'connected', 'dest': 'ready'},
        {'trigger': 'terminate', 'source': 'ready', 'dest': 'connected'},
        {'trigger': 'run', 'source': 'ready', 'dest': 'running', 'conditions': 'code_loaded'},
        {'trigger': 'stop', 'source': 'running', 'dest': 'ready'},
        {'trigger': 'pause', 'source': 'running', 'dest': 'paused'},
        {'trigger': 'resume', 'source': 'paused', 'dest': 'running'},
        {'trigger': 'stop', 'source': 'paused', 'dest': 'ready'},
        {'trigger': 'reset', 'source': '*', 'dest': 'idle'}
    ]

    def __init__(self):
        self.machine = Machine(model=self, states=ExerciseManager.states, transitions=ExerciseManager.transitions,
                               initial='idle')

    def on_enter_connected(self):
        print("Connect state entered")

    def on_enter_ready(self):
        print("Start state entered")

    def code_loaded(self):
        return True