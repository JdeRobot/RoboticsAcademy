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
from gui.generator import Generator
from gui.cmakevars import CMAKE_INSTALL_PREFIX
from gui.transitiontype import TransitionType
import os

class PythonGenerator(Generator):
    def __init__(self, libraries, config, interfaceHeaders, states):
        Generator.__init__(self)
        self.libraries = libraries
        self.config = config
        self.interfaceHeaders = interfaceHeaders
        self.states = states

    def getAllStates(self):
        addedStates = {}
        allStates = []
        for state in self.states:
            if state.id not in addedStates:
                addedStates[state.id] = state
                allStates.append(state)

            for childState in state.getChildren():
                if childState.id not in addedStates:
                    addedStates[childState.id] = childState
                    allStates.append(childState)

        return allStates

    def getAllTransitions(self):
        addedTransitions = {}
        transitions = []
        for state in self.states:
            for tran in state.getOriginTransitions():
                if tran.id not in addedTransitions:
                    addedTransitions[tran.id] = tran
                    transitions.append(tran)
            for childState in state.getChildren():
                for tran in childState.getOriginTransitions():
                    if tran.id not in addedTransitions:
                        addedTransitions[tran.id] = tran
                        transitions.append(tran)

        return transitions

    def generate(self, projectPath, projectName):
        stringList = []
        self.generateImports(stringList)
        self.generateStateClasses(stringList)
        self.generateTransitionClasses(stringList)
        self.generateInterfaces(stringList, projectName)
        self.generateMain(stringList)
        sourceCode = ''.join(stringList)
        fp = open(projectPath + os.sep + projectName + '.py', 'w')
        fp.write(sourceCode)
        fp.close()

        self.generateAndSaveCfgYaml(projectPath, projectName)

        os.system('chmod +x ' + projectPath + os.sep + projectName + '.py')


    def generateImports(self, headerStr):
        mystr = '''#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys, threading, time, signal
'''
        headerStr.append(mystr)
        headerStr.append('sys.path.append("' + CMAKE_INSTALL_PREFIX + '/lib/python2.7")\n')
        headerStr.append('sys.path.append("' + CMAKE_INSTALL_PREFIX + '/lib/python2.7/visualStates_py")\n')
        mystr = '''from codegen.python.state import State
from codegen.python.temporaltransition import TemporalTransition
from codegen.python.conditionaltransition import ConditionalTransition
from codegen.python.runtimegui import RunTimeGui
from PyQt5.QtWidgets import QApplication
import config, comm
'''
        headerStr.append(mystr)
        for lib in self.libraries:
            headerStr.append('import ')
            headerStr.append(lib)
            headerStr.append('\n')
        headerStr.append('\n')

        # for cfg in self.configs:
        #     headerStr.append('from jderobot import ')
        #     headerStr.append(cfg['interface'])
        #     headerStr.append('Prx\n')

        headerStr.append('\n')

        return headerStr

    def generateStateClasses(self, stateStr):
        for state in self.getAllStates():
            self.generateStateClass(state, stateStr)

    def generateStateClass(self, state, stateStr):
        stateStr.append('class State')
        stateStr.append(str(state.id))
        stateStr.append('(State):\n')

        stateStr.append('\tdef __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):\n')
        stateStr.append('\t\tState.__init__(self, id, initial, cycleDuration, parent, gui)\n')
        stateStr.append('\t\tself.interfaces = interfaces\n\n')

        stateStr.append('\tdef runCode(self):\n')
        if len(state.getCode()) > 0:
            for codeLine in state.getCode().split('\n'):
                stateStr.append('\t\t' + codeLine + '\n')
        else:
            stateStr.append('\t\tpass\n')
        stateStr.append('\n')


    def generateInterfaces(self, interfaceStr, projectName):
        mystr = '''class Interfaces():
\tdef __init__(self):
\t\tself.jdrc = None
'''
        interfaceStr.append(mystr)
        for cfg in self.config.getInterfaces():
            interfaceStr.append('\t\tself.' + cfg['name'] + ' = None\n')

        for state in self.getAllStates():
            if len(state.getVariables()) > 0:
                for varLine in state.getVariables().split('\n'):
                    varLine = varLine.strip()
                    if len(varLine) > 0:
                        interfaceStr.append('\t\t' + varLine + '\n')

        interfaceStr.append('\n')

        interfaceStr.append('\t\tself.connectProxies()\n\n')

        interfaceStr.append('\tdef connectProxies(self):\n')
        interfaceStr.append('\t\tcfg = config.load(sys.argv[1])\n')
        interfaceStr.append('\t\tself.jdrc = comm.init(cfg, "' + projectName + '")\n')

        for cfg in self.config.getInterfaces():
            interfaceStr.append('\t\tself.' + cfg['name'] + ' = self.jdrc.get'+ cfg['interface']+'Client("'+projectName+'.' + cfg['name'] + '")\n')
            interfaceStr.append('\t\tif not self.' + cfg['name'] + ':\n')
            interfaceStr.append('\t\t\traise Exception("could not create client with name:' + cfg['name'] + '")\n')
            interfaceStr.append('\t\tprint("' + cfg['name'] + ' is connected")\n')

        interfaceStr.append('\n')

        interfaceStr.append('\tdef destroyProxies(self):\n')
        interfaceStr.append('\t\tif self.jdrc is not None:\n')
        interfaceStr.append('\t\t\tself.jdrc.destroy()\n\n')

        for state in self.getAllStates():
            if len(state.getFunctions()) > 0:
                for funcLine in state.getFunctions().split('\n'):
                    interfaceStr.append('\t' + funcLine + '\n')
                interfaceStr.append('\n')

    def generateTransitionClasses(self, tranStr):
        for tran in self.getAllTransitions():
            if tran.getType() == TransitionType.CONDITIONAL:
                tranStr.append('class Tran' + str(tran.id) + '(ConditionalTransition):\n')
                tranStr.append('\tdef __init__(self, id, destinationId, interfaces):\n')
                tranStr.append('\t\tConditionalTransition.__init__(self, id, destinationId)\n')
                tranStr.append('\t\tself.interfaces = interfaces\n\n')
                tranStr.append('\tdef checkCondition(self):\n')
                for checkLine in tran.getCondition().split('\n'):
                    tranStr.append('\t\t' + checkLine + '\n')
                tranStr.append('\n')
            elif tran.getType() == TransitionType.TEMPORAL:
                tranStr.append('class Tran' + str(tran.id) + '(TemporalTransition):\n\n')
            tranStr.append('\tdef runCode(self):\n')
            if len(tran.getCode()) > 0:
                for codeLine in tran.getCode().split('\n'):
                    tranStr.append('\t\t' + codeLine + '\n')
                tranStr.append('\n')
            else:
                tranStr.append('\t\tpass\n\n')

    def generateMain(self, mainStr):
        mystr = '''displayGui = False
guiThread = None
gui = None
'''
        mainStr.append(mystr)
        for state in self.states:
            mainStr.append('state' + str(state.id) + ' = None\n')
        mainStr.append('\n')

        mystr = '''def signal_handler(signal, frame):
\tglobal gui
\tprint("SIGINT is captured. The program exits")
\tif gui is not None:
\t\tgui.close()
'''
        mainStr.append(mystr)
        for state in self.states:
            mainStr.append('\tglobal state' + str(state.id) + '\n')
            mainStr.append('\tstate' + str(state.id) + '.stop()\n')
        mainStr.append('\n')

        mystr = '''def readArgs():
\tglobal displayGui
\tfor arg in sys.argv:
\t\tsplitedArg = arg.split('=')
\t\tif splitedArg[0] == '--displaygui':
\t\t\tif splitedArg[1] == 'True' or splitedArg[1] == 'true':
\t\t\t\tdisplayGui = True
\t\t\t\tprint('runtime gui enabled')
\t\t\telse:
\t\t\t\tdisplayGui = False
\t\t\t\tprint('runtime gui disabled')

def runGui():
\tglobal gui
\tapp = QApplication(sys.argv)
\tgui = RunTimeGui()
\tgui.show()
\tapp.exec_()

'''
        mainStr.append(mystr)

        mainStr.append('if __name__ == "__main__":\n')
        mainStr.append('\tinterfaces = Interfaces()\n\n')
        mainStr.append('\treadArgs()\n')
        mainStr.append('\tif displayGui:\n')
        mainStr.append('\t\tguiThread = threading.Thread(target=runGui)\n')
        mainStr.append('\t\tguiThread.start()\n\n')

        mainStr.append('\n\tif displayGui:\n')
        mainStr.append('\t\twhile(gui is None):\n')
        mainStr.append('\t\t\ttime.sleep(0.1)\n\n')
        # create runtime gui code
        for state in self.getAllStates():
            mainStr.append('\t\tgui.addState(' + str(state.id) + ', "' + state.name +
                           '", ' + str(state.initial) + ', ' + str(state.x) + ', ' + str(state.y))
            if state.parent is None:
                mainStr.append(', None)\n')
            else:
                mainStr.append(', ' + str(state.parent.id) +')\n')

        mainStr.append('\n')

        for tran in self.getAllTransitions():
            mainStr.append('\t\tgui.addTransition(' + str(tran.id) + ', "' + tran.name + '", ' +
                           str(tran.origin.id) + ', ' + str(tran.destination.id) +
                           ', ' + str(tran.x) + ', ' + str(tran.y) + ')\n')
        mainStr.append('\n')

        mainStr.append('\tif displayGui:\n')
        mainStr.append('\t\tgui.emitLoadFromRoot()\n')
        mainStr.append('\t\tgui.emitActiveStateById(0)\n\n')

        for state in self.getAllStates():
            mainStr.append('\tstate' + str(state.id) + ' = State' + str(state.id) +
                           '(' + str(state.id) + ', ' + str(state.initial) + ', interfaces, ' +
                           str(state.getTimeStep()))
            if state.parent is None:
                mainStr.append(', None, gui)\n')
            else:
                mainStr.append(', state' + str(state.parent.id) + ', gui)\n')
        mainStr.append('\n')

        # create and add transitions to their origins
        for tran in self.getAllTransitions():
            if tran.getType() == TransitionType.TEMPORAL:
                mainStr.append('\ttran' + str(tran.id) + ' = Tran' + str(tran.id) +
                               '(' + str(tran.id) + ', ' + str(tran.destination.id) + ', ' + str(tran.getTemporalTime()) + ')\n')
            elif tran.getType() == TransitionType.CONDITIONAL:
                mainStr.append('\ttran' + str(tran.id) + ' = Tran' + str(tran.id) +
                               '(' + str(tran.id) + ', ' + str(tran.destination.id) + ', interfaces)\n')

            mainStr.append('\tstate' + str(tran.origin.id) + '.addTransition(tran' + str(tran.id) + ')\n\n')

        mainStr.append('\ttry:\n')
        # start threads
        for state in self.states:
            mainStr.append('\t\tstate' + str(state.id) + '.startThread()\n')
        mainStr.append('\t\tsignal.signal(signal.SIGINT, signal_handler)\n')
        mainStr.append('\t\tsignal.pause()\n')
        # join threads
        for state in self.states:
            mainStr.append('\t\tstate' + str(state.id) + '.join()\n')

        mainStr.append('\t\tif displayGui:\n')
        mainStr.append('\t\t\tguiThread.join()\n\n')

        mainStr.append('\t\tinterfaces.destroyProxies()\n')
        mainStr.append('\texcept:\n')
        for state in self.states:
            mainStr.append('\t\tstate' + str(state.id) + '.stop()\n')
        mainStr.append('\t\tif displayGui:\n')
        mainStr.append('\t\t\tgui.close()\n')
        mainStr.append('\t\t\tguiThread.join()\n\n')
        # join threads
        for state in self.states:
            mainStr.append('\t\tstate' + str(state.id) + '.join()\n')
        mainStr.append('\t\tinterfaces.destroyProxies()\n')
        mainStr.append('\t\tsys.exit(1)\n')