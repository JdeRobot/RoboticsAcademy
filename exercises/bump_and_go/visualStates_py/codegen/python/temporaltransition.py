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
from codegen.python.transition import Transition
from time import time

class TemporalTransition(Transition):
    def __init__(self, id,  destinationId, elapsedTime):
        super(TemporalTransition, self).__init__(id, destinationId)
        # elapsed time in milliseconds
        self.elapsedTime = elapsedTime
        self.startTime = None

    def init(self):
        self.startTime = time()*1000

    def runCode(self):
        pass

    def checkCondition(self):
        Transition.checkCondition(self)
        diffTime = (time()*1000)-self.startTime
        if diffTime > self.elapsedTime:
            return True
        else:
            return False
