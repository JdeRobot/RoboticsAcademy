#
#  Copyright (C) 1997-2015 JDE Developers Team
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
#       Alberto Martin Florido <almartinflorido@gmail.com>
#

class ColorFilterValues:

    def __init__(self):
        self.hmin = 0
        self.hmax = 0
        self.vmin = 0
        self.vmax = 0
        self.smin = 0
        self.smax = 0

    def getHMin(self):
        return self.hmin

    def setHMin(self,value):
        self.hmin=value

    def getHMax(self):
        return self.hmax

    def setHMax(self,value):
        self.hmax=value

    def getSMin(self):
        return self.smin

    def setSMin(self,value):
        self.smin=value

    def getSMax(self):
        return self.smax

    def setSMax(self,value):
        self.smax=value

    def getVMin(self):
        return self.vmin

    def setVMin(self,value):
        self.vmin=value

    def getVMax(self):
        return self.vmax

    def setVMax(self,value):
        self.vmax=value

