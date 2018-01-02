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

import os
# a class to discover JdeRobot and ROS interfaces
from gui.cmakevars import CMAKE_INSTALL_PREFIX

class Interfaces:

    interfaces = None

    @staticmethod
    def getInterfaces():
        if Interfaces.interfaces is None:
            os.system(CMAKE_INSTALL_PREFIX + '/bin/getinterfaces.sh ' + CMAKE_INSTALL_PREFIX + '/include/jderobot/comm/interfaces > /tmp/allInterfaces.txt')
            fp = open('/tmp/allInterfaces.txt')
            Interfaces.interfaces = {}
            for line in fp:
                data = line.strip("\n").split(' ')
                Interfaces.interfaces[data[0]] = data[1]

        return Interfaces.interfaces

    @staticmethod
    def getRosMessageTypes(rosDir = '/opt/ros/kinetic'):
        messageDir = rosDir + '/include'
        allContents = os.listdir(messageDir)
        messages = []
        for entry in allContents:
            if os.path.isdir(messageDir + '/' + entry):
                if entry.find('_msgs') >= 0:
                    messages.append(entry)

        types = []
        for msg in messages:
            typeDir = msg
            for entry in os.listdir(messageDir + '/' + msg):
                if os.path.isfile(messageDir + '/' + msg + '/' + entry):
                    if entry.find('.h') >= 0 and entry[0].isupper():
                        type = {}
                        type['typeDir'] = typeDir
                        type['type'] = entry[:entry.find('.h')]
                        types.append(type)

        return types


if __name__ == '__main__':
    types = Interfaces.getRosMessageTypes()
    for type in types:
        print(type['typeDir'] + '::' + type['type'])
