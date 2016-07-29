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
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#


import traceback
import jderobot
import threading
import Ice


class Motors:

    def __init__(self, ic, prefix):
        self.lock = threading.Lock()
        self.v = self.w = 0
        prop = ic.getProperties()
        
        self.maxW = prop.getProperty(prefix+".maxW")
        if not self.maxW:
            self.maxW = 0.5
            print (prefix+".maxW not provided, the default value is used: "+ repr(self.maxW))
                

        self.maxV = prop.getProperty(prefix+".maxV")
        if not self.maxV:
            self.maxV = 5
            print (prefix+".maxV not provided, the default value is used: "+ repr(self.maxV))

        try:
            base = ic.propertyToProxy(prefix+".Proxy")
            self.proxy = jderobot.MotorsPrx.checkedCast(base)


            if not self.proxy:
                print ('Interface Motors not configured')

        except Ice.ConnectionRefusedException:
            print('Motors: connection refused')
        except:
            traceback.print_exc()
            exit(-1)

    def setV(self, v):
        self.v = v

    def setW(self, w):
        self.w = w

    def getMaxW(self):
        return self.maxW

    def getMaxV(self):
        return self.maxV

    def sendVelocities(self):
        if hasattr(self,"proxy") and self.proxy:
            self.sendV(self.v)
            self.sendW(self.w)

    def sendV(self, v):
        if hasattr(self,"proxy") and self.proxy:
            self.lock.acquire()
            self.proxy.setV(v)
            self.lock.release()

    def sendW(self, w):
        if hasattr(self,"proxy") and self.proxy:
            self.lock.acquire()
            self.proxy.setW(w)
            self.lock.release()
