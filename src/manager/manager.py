from __future__ import annotations

import asyncio

from transitions.extensions.asyncio import AsyncMachine

from src.comms.consumer import ManagerConsumer
from src.manager.launcher.launcher_engine import LauncherEngine


class Manager:
    states = [
        "idle",
        "connected",
        "ready",
        "running",
        "paused"
    ]

    transitions = [
        # Transitions for state idle
        {'trigger': 'connect', 'source': 'idle', 'dest': 'connected'},
        # Transitions for state connected
        {'trigger': 'launch', 'source': 'connected', 'dest': 'ready', 'before': 'on_launch'},
        # Transitions for state ready
        {'trigger': 'terminate', 'source': 'ready', 'dest': 'connected', 'before': 'on_terminate'},
        {'trigger': 'load', 'source': 'ready', 'dest': None, 'before': 'load_code'},
        {'trigger': 'run', 'source': 'ready', 'dest': 'running', 'conditions': 'code_loaded'},
        # Transitions for state running
        {'trigger': 'stop', 'source': 'running', 'dest': 'ready'},
        {'trigger': 'pause', 'source': 'running', 'dest': 'paused'},
        # Transitions for state paused
        {'trigger': 'resume', 'source': 'paused', 'dest': 'running'},
        {'trigger': 'stop', 'source': 'paused', 'dest': 'ready'},
        # Global transitions
        {'trigger': 'reset', 'source': '*', 'dest': 'idle'}
    ]

    def __init__(self, consumer: ManagerConsumer):
        self.machine = AsyncMachine(model=self, states=Manager.states, transitions=Manager.transitions,
                                    initial='idle', send_event=True, after_state_change=self.state_change)
        self.consumer = consumer
        self.launcher = None

    async def state_change(self, event):
        print(f"State changed to {self.state}")
        if self.consumer is not None:
            await self.consumer.send_message({'state': self.state})

    def on_launch(self, event):
        def terminated_callback(name, code):
            print(f"Manager: {name} died with code {code}")
            # asyncio.ensure_future(self.terminate())

        configuration = event.kwargs.get('data', {})

        if "launch" not in configuration.keys():
            raise Exception("Launch configuration missing")

        print(f"Launch transition started, configuration: {configuration}")
        configuration['terminated_callback'] = terminated_callback
        self.launcher = LauncherEngine(**configuration)
        self.launcher.run()

    def on_terminate(self, event):
        self.launcher.terminate()

    def on_enter_connected(self, event):
        print("Connect state entered")

    def on_enter_ready(self, event):
        configuration = event.kwargs.get('data', {})
        print(f"Start state entered, configuration: {configuration}")

    def load_code(self, event):
        print("Internal transition load_code executed")

    def code_loaded(self):
        return True
