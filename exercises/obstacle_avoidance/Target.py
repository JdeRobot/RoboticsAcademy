#
#  Copyright (C) 1997-2016 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Eduardo Perdices <eperdices@gsyc.es>
#

import sys, traceback

class Target:
    def __init__(self,id,pose,active=False,reached=False):
        self.id=id
        self.pose=pose
        self.active=active
        self.reached=reached

    def getPose(self):
        return self.pose

    def getId(self):
        return self.id

    def getPose(self):
        return self.pose

    def isReached(self):
        return self.reached

    def setReached(self,value):
        self.reached=value

    def isActive(self):
        return self.active

    def setActive(self,value):
        self.active=True

