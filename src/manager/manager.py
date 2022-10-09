import asyncio

from transitions import Machine

from src.manager.launcher.launcher import Launcher


class Manager:
    states = [
        "idle",
        "connected",
        "ready",
        "running",
        "paused"
    ]

    transitions = [
        {'trigger': 'connect', 'source': 'idle', 'dest': 'connected'},
        {'trigger': 'launch', 'source': 'connected', 'dest': 'ready', 'before': 'on_launch'},
        {'trigger': 'terminate', 'source': 'ready', 'dest': 'connected'},
        {'trigger': 'run', 'source': 'ready', 'dest': 'running', 'conditions': 'code_loaded'},
        {'trigger': 'stop', 'source': 'running', 'dest': 'ready'},
        {'trigger': 'pause', 'source': 'running', 'dest': 'paused'},
        {'trigger': 'resume', 'source': 'paused', 'dest': 'running'},
        {'trigger': 'stop', 'source': 'paused', 'dest': 'ready'},
        {'trigger': 'reset', 'source': '*', 'dest': 'idle'}
    ]

    def __init__(self, state_change_handler=None):
        self.state_change_handler = state_change_handler
        self.machine = Machine(model=self, states=Manager.states, transitions=Manager.transitions,
                               initial='idle', send_event=True, after_state_change=self.state_change)

    def state_change(self, event):
        if self.state_change_handler is not None:
            loop = asyncio.get_event_loop()
            loop.run_until_complete(self.state_change_handler(self.state))

    def on_launch(self, event):
        configuration = event.kwargs.get('data', {})

        if "launch" not in configuration.keys() or True:
            raise Exception("Launch configuration missing")

        print(f"Launch transition started, configuration: {configuration}")
        launcher = Launcher(configuration['launch'])
        launcher.run()

    def on_enter_connected(self, event):
        print("Connect state entered")

    def on_enter_ready(self, event):
        configuration = event.kwargs.get('data', {})
        print(f"Start state entered, configuration: {configuration}")

    def code_loaded(self):
        return True