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
from xml.dom import minidom
from gui.state import State
from gui.config import ROS, JDEROBOTCOMM, RosConfig, JdeRobotConfig
import os

class FileManager():
    def __init__(self):
        self.fullPath = ""

    def getFullPath(self):
        return self.fullPath

    def setFullPath(self, path):
        # check for the xml extensition if missing add the extension
        if path.rfind('.xml') < 0:
            path += '.xml'
        self.fullPath = path

    def save(self, rootState, config, libraries):
        doc = minidom.Document()
        root = doc.createElement('VisualStates')
        doc.appendChild(root)

        # save config data
        if config is not None:
            root.appendChild(config.createNode(doc))

        # save libraries
        libraryElement = doc.createElement('libraries')
        for lib in libraries:
            libElement = doc.createElement('library')
            libElement.appendChild(doc.createTextNode(lib))
            libraryElement.appendChild(libElement)
        root.appendChild(libraryElement)

        root.appendChild(self.createDocFromState(rootState, doc))
        xmlStr = doc.toprettyxml(indent='  ')
        with open(self.fullPath, 'w') as f:
            f.write(xmlStr)

    def createDocFromState(self, state, doc):
        stateElement = state.createElement(doc)
        return stateElement

    def open(self, fullPath):
        self.setFullPath(fullPath)
        doc = minidom.parse(fullPath)
        rootNode = doc.getElementsByTagName('VisualStates')[0].getElementsByTagName('state')[0]
        rootState = State(0, 'root', True)
        rootState.parse(rootNode)

        # parse configs
        config = None
        if len(doc.getElementsByTagName('VisualStates')[0].getElementsByTagName('config')) > 0:
            configElement = doc.getElementsByTagName('VisualStates')[0].getElementsByTagName('config')[0]
            if configElement.getAttribute('type') == str(ROS):
                config = RosConfig()
                config.loadNode(configElement)
                config.type = ROS
            elif configElement.getAttribute('type') == str(JDEROBOTCOMM):
                config = JdeRobotConfig()
                config.loadNode(configElement)
                config.type = JDEROBOTCOMM

        libraries = []

        # parse libraries
        libraryElements = doc.getElementsByTagName('VisualStates')[0].getElementsByTagName('libraries')
        if len(libraryElements) > 0:
            libraryElements = libraryElements[0].getElementsByTagName('library')
            for libElement in libraryElements:
                libraries.append(libElement.childNodes[0].nodeValue)

        return (rootState, config, libraries)

    def hasFile(self):
        return len(self.fullPath) > 0

    def getFileName(self):
        name = ''
        if self.fullPath.rfind(os.sep) >= 0:
            name = self.fullPath[self.fullPath.rfind(os.sep)+1:len(self.fullPath)]

        # remove the file extension
        if len(name) > 0:
            name = name[0:name.rfind('.')]

        return name

    def getPath(self):
        path = ''
        if self.fullPath.rfind(os.sep) >= 0:
            path = self.fullPath[0:self.fullPath.rfind(os.sep)]
        return path






