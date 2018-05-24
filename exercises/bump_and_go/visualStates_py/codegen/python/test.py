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
from codegen.python.state import State
from codegen.python.temporaltransition import TemporalTransition
from codegen.python.transition import Transition
import easyiceconfig as EasyIce
import sys
from jderobot import MotorsPrx

class State0(State):
    def __init__(self, id, initial, config, cycleDuration, parent=None):
        super(State, self).__init__(id, initial, config, cycleDuration, parent)
        
        pass
    def runCode(self):
        print('state0 run code')

class State1(State):
    def runCode(self):
        print('state1 run code')

class State2(State):
    def runCode(self):
        print('state2 run code')

class State3(State):
    def runCode(self):
        print('state3 run code')

class State4(State):
    def runCode(self):
        print('state4 run code')
        
class Tran0(Transition):
    def checkCondition(self):
        print('check condition')
    
    def runCode(self):
        print('run code')

class Interfaces():
    def __init__(self):
        self.myMotors = None
        self.ic = None
        self.connectProxies()

    def connectProxies(self):
        self.ic = EasyIce.initialize(sys.argv)
        self.myMotors = self.ic.propertyToProxy("automata.myMotors.Proxy")
        if not self.myMotors:
            raise Exception("could not create proxy with myMotors")
        self.myMotors = MotorsPrx.checkedCast(self.myMotors)
        if not self.myMotors:
            raise Exception("invalid proxy automata.myMotors.Proxy")

    def destroyProxies(self):
        if self.ic is not None:
            self.ic.destroy()


if __name__ == '__main__':
    # create easy ice config connections
    interfaces = Interfaces()

    state0 = State0(0, True, interfaces, 100)
    state1 = State1(1, True, interfaces, 100, state0)
    state2 = State2(2, False, interfaces, 100, state0)

    state3 = State3(3, True, interfaces, 100, state1)
    state4 = State4(4, False, interfaces, 100, state1)

    tran0 = TemporalTransition(0, 2, 3000)
    tran1 = TemporalTransition(1, 1, 3000)

    state1.addTransition(tran0)
    state2.addTransition(tran1)

    tran2 = TemporalTransition(2, 4, 1000)
    tran3 = TemporalTransition(3, 3, 1000)

    state3.addTransition(tran2)
    state4.addTransition(tran3)

    # start threads
    state0.startThread()
    state1.startThread()

    state0.join()
    state1.join()




