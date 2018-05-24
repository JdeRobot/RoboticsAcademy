'''
   Copyright (C) 1997-2017 JDERobot Developers Team

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Library General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, see <http://www.gnu.org/licenses/>.

   Authors : Okan Asik (asik.okan@gmail.com)

  '''
from threading import Thread
import time, sys

class State():
    def __init__(self, id, initial, cycleDuration, parent=None, gui=None):
        self.id = id
        self.active = False
        self.thread = None
        self.running = False
        self.parent = parent
        self.currentState = None
        self.initial = initial
        self.displayGui = False

        self.cycleDuration = cycleDuration

        self.states = []
        self.transitions = []
        self.statesById = {}

        self.gui = gui

        # add state to parent
        if self.parent is not None:
            self.parent.addState(self)

    def init(self):
        for tran in self.transitions:
            tran.init()
        if self.gui is not None:
            self.gui.emitRunningStateById(self.id)

    def runCode(self):
        pass

    def addState(self, state):
        if state.initial:
            self.currentState = state

        self.states.append(state)
        self.statesById[state.id] = state

    def addTransition(self, tran):
        self.transitions.append(tran)

    def startThread(self):
        self.running = True
        self.thread = Thread(target=self.run)
        self.thread.start()

    def run(self):
        initState = True
        while(self.running):
            startTime = self.getCurrentTime()

            runState = False
            if self.parent is not None:
                if self.parent.currentState == self:
                    runState = True
            elif self.parent is None:
                runState = True

            if initState:
                self.currentState.init()
                initState = False

            if runState:
                # transition evaluations
                for tran in self.currentState.transitions:
                    if tran.checkCondition():
                        tran.runCode()
                        self.currentState = self.statesById[tran.getDestinationId()]
                        self.currentState.init()
                        initState = False
                        break

                # print('current state:' + str(self.currentState.id))
                self.currentState.runCode()

            # control the cycle running time
            finishTime = self.getCurrentTime()
            # calculate elapsed time in milliseconds
            elapsedTime = (finishTime - startTime)/1000

            if elapsedTime < self.cycleDuration:
                elapsedTime = self.cycleDuration - elapsedTime
                # sleep in seconds
                time.sleep(elapsedTime / 1000)

    def stop(self):
        self.running = False

    # returns the current time in microseconds
    def getCurrentTime(self):
        return time.time() * 1000000

    def join(self):
        self.thread.join()
