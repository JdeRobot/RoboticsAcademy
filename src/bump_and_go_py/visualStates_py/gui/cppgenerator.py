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
from gui.transitiontype import TransitionType
from gui.generator import Generator
from gui.cmakevars import CMAKE_INSTALL_PREFIX
from gui.cppparser import CPPParser
import os, stat

class CppGenerator(Generator):
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
        self.generateHeaders(stringList, projectName)
        self.generateInterfaceClass(stringList)
        self.generateStateClasses(stringList)
        self.generateTransitionClasses(stringList)
        stringList.append('#endif')
        sourceCode = ''.join(stringList)
        fp = open(projectPath + os.sep + projectName + '.h', 'w')
        fp.write(sourceCode)
        fp.close()

        stringList = []
        self.generateHeadersForCpp(stringList, projectName)
        self.generateStateMethods(stringList)
        self.generateTranMethods(stringList)
        self.generateInterfaceMethods(stringList, projectName)
        self.generateReadArgs(stringList, projectName)
        self.generateMain(stringList, projectName)
        sourceCode = ''.join(stringList)
        fp = open(projectPath + os.sep + projectName + '.cpp', 'w')
        fp.write(sourceCode)
        fp.close()

        stringList = []
        self.generateRunTimeGui(stringList)
        sourceCode = ''.join(stringList)
        fp = open(projectPath + os.sep + projectName + '_runtime.py', 'w')
        fp.write(sourceCode)
        fp.close()
        # make runtime gui python file executable
        os.chmod(projectPath + os.sep + projectName + '_runtime.py', stat.S_IEXEC | stat.S_IXOTH | stat.S_IWRITE | stat.S_IREAD)

        # stringList = []
        # self.generateCfg(stringList)
        # cfgString = ''.join(stringList)
        # fp = open(projectPath + os.sep + projectName + '.cfg', 'w')
        # fp.write(cfgString)
        # fp.close()
        self.generateAndSaveCfgYaml(projectPath, projectName)

        stringList = []
        self.generateCmake(stringList, projectName)
        cmakeString = ''.join(stringList)
        fp = open(projectPath + os.sep + 'CMakeLists.txt', 'w')
        fp.write(cmakeString)
        fp.close()


    def generateHeaders(self, headers, projectName):
        headers.append('#ifndef ' + projectName + '_H\n')
        headers.append('#define ' + projectName + '_H\n\n')

        headers.append('#include <state.h>\n')
        headers.append('#include <temporaltransition.h>\n')
        headers.append('#include <conditionaltransition.h>\n')
        headers.append('#include <easyiceconfig/EasyIce.h>\n\n')

        for lib in self.libraries:
            headers.append('#include <')
            headers.append(lib.strip('\n'))
            headers.append('>\n')

        # generate interface headers
        for cfg in self.config.getInterfaces():
            headers.append('#include <jderobot/config/config.h>\n')
            headers.append('#include <jderobot/comm/communicator.hpp>\n')
            headers.append('#include <jderobot/comm/')
            headers.append(self.interfaceHeaders[cfg['interface']].strip('\n'))
            headers.append('.hpp>\n')

        headers.append('\n')

        return headers

    def generateStateClasses(self, classStr):
        for state in self.getAllStates():
            classStr.append('class State' + str(state.id) + ' : public State {\n')
            classStr.append('public:\n')
            classStr.append('\tInterfaces* interfaces;\n')
            classStr.append('\tState' + str(state.id) + '(int id, bool initial, Interfaces* interfaces, int cycleDuration, State* parent, RunTimeGui* gui):\n')
            classStr.append('\t\tState(id, initial, cycleDuration, parent, gui) {this->interfaces = interfaces;}\n')
            classStr.append('\tvirtual void runCode();\n')
            classStr.append('};\n\n')


    def generateTransitionClasses(self, classStr):
        for tran in self.getAllTransitions():
            if tran.getType() == TransitionType.CONDITIONAL:
                classStr.append('class Tran' + str(tran.id) + ' : public ConditionalTransition {\n')
                classStr.append('\tpublic:\n')
                classStr.append('\tInterfaces* interfaces;')
                classStr.append('\tTran' + str(tran.id) + '(int id, int destId, Interfaces* interfaces):\n')
                classStr.append('ConditionalTransition(id, destId) {this->interfaces = interfaces;}\n')
                classStr.append('\tvirtual void init();\n')
                classStr.append('\tvirtual bool checkCondition();\n')
                classStr.append('\tvirtual void runCode();\n')
                classStr.append('};\n\n')
            elif tran.getType() == TransitionType.TEMPORAL:
                classStr.append('class Tran' + str(tran.id) + ' : public TemporalTransition {\n')
                classStr.append('\tpublic:\n')
                classStr.append('\tTran' + str(tran.id) + '(int id, int destId, int elapsedTime):\n')
                classStr.append('TemporalTransition(id, destId, elapsedTime) {}\n')
                classStr.append('\tvirtual void runCode();\n')
                classStr.append('};\n\n')


    def generateInterfaceClass(self, classStr):
        classStr.append('class Interfaces {\n')
        classStr.append('public:\n')
        classStr.append('\tComm::Communicator* jdrc;\n')
        for cfg in self.config.getInterfaces():
            classStr.append('\tComm::' + cfg['interface'] + 'Client* ' + cfg['name'] + ';\n')
        classStr.append('\n')
        classStr.append('\tvirtual void connectProxies(int argc, char* argv[]);\n')
        classStr.append('\tvirtual void destroyProxies();\n')
        for state in self.getAllStates():
            # define variables
            types, varNames, initialValues = CPPParser.parseVariables(state.getVariables())
            for i in range(len(types)):
                classStr.append('\t' + types[i] + ' ' + varNames[i] + ';\n')
            classStr.append('\n')
            returnTypes, funcNames, codes = CPPParser.parseFunctions(state.getFunctions())
            for i in range(len(returnTypes)):
                classStr.append('\t' + returnTypes[i] + ' ' + funcNames[i] + ';\n')
        classStr.append('};\n\n')


    def generateHeadersForCpp(self, headerStr, projectName):
        headerStr.append('#include "' + projectName + '.h"\n')
        headerStr.append('#include <iostream>\n')
        headerStr.append('#include <string>\n')
        headerStr.append('#include <runtimegui.h>\n\n')

    def generateStateMethods(self, stateStr):
        for state in self.getAllStates():
            stateStr.append('void State' + str(state.id) + '::runCode() {\n')
            for codeLine in state.getCode().split('\n'):
                stateStr.append('\t' + codeLine + '\n')
            stateStr.append('}\n\n')


    def generateTranMethods(self, tranStr):
        for tran in self.getAllTransitions():
            if tran.getType() == TransitionType.CONDITIONAL:
                #todo: currently user does not provide init method
                tranStr.append('void Tran' + str(tran.id) + '::init() {\n')
                tranStr.append('}\n\n')
                tranStr.append('bool Tran' + str(tran.id) + '::checkCondition() {\n')
                for codeLine in tran.getCondition().split('\n'):
                    tranStr.append('\t' + codeLine + '\n')
                tranStr.append('}\n')

            tranStr.append('void Tran' + str(tran.id) + '::runCode() {\n')
            for codeLine in tran.getCode().split('\n'):
                tranStr.append('\t' + codeLine + '\n')
            tranStr.append('}\n\n')


    def generateInterfaceMethods(self, proxyStr, projectName):
        proxyStr.append('void Interfaces::connectProxies(int argc, char* argv[]) {\n')
        proxyStr.append('\tConfig::Properties props = Config::load(argc, argv);\n')
        proxyStr.append('\tjdrc = new Comm::Communicator(props);\n\n')
        for cfg in self.config.getInterfaces():
            proxyStr.append('\t' + cfg['name'] + ' = Comm::get' + cfg['interface'] + 'Client(jdrc, "'+projectName+'.' + cfg['name'] + '");\n')
            proxyStr.append('\tif (' + cfg['name'] + ' == NULL) {\n')
            proxyStr.append('\t\tthrow "invalid proxy ' + cfg['name'] + '";\n')
            proxyStr.append('\t}\n')
            proxyStr.append('\tstd::cout << "' + cfg['name'] + ' is connected" << std::endl;\n\n')

        # set inital values of variables
        for state in self.getAllStates():
            types, varNames, initialValues = CPPParser.parseVariables(state.getVariables())
            for i in range(len(types)):
                if initialValues[i] is not None:
                    proxyStr.append('\t' + varNames[i] + ' = ' + initialValues[i] + ';\n')

        proxyStr.append('}\n\n')

        proxyStr.append('void Interfaces::destroyProxies() {\n')
        proxyStr.append('\tif (jdrc != 0) {\n')
        proxyStr.append('\t}\n}\n\n')

        for state in self.getAllStates():
            returnTypes, funcNames, codes = CPPParser.parseFunctions(state.getFunctions())
            for i in range(len(returnTypes)):
                proxyStr.append(returnTypes[i] + ' Interfaces::' + funcNames[i] + '\n')
                proxyStr.append(codes[i])
                proxyStr.append('\n\n')


    def generateReadArgs(self, argStr, projectName):
        mystr = '''
pthread_t guiThread;    
RunTimeGui* runTimeGui = NULL;
bool displayGui = false;

void readArgs(int *argc, char* argv[]) {
\tint i;
\tstd::string splitedArg;

\tfor(i = 0; i < *argc; i++) {
\t\tsplitedArg = strtok(argv[i], "=");
\t\tif (splitedArg.compare("--displaygui") == 0){
\t\t\tsplitedArg = strtok(NULL, "=");
\t\t\tif (splitedArg.compare("true") == 0 || splitedArg.compare("True") == 0){
\t\t\t\tdisplayGui = true;
\t\t\t\tstd::cout << "displayGui ENABLED" << std::endl;
\t\t\t}else{
\t\t\t\tdisplayGui = false;
\t\t\t\tstd::cout << "displayGui DISABLED" << std::endl;
\t\t\t}
\t\t}
\t\tif(i == *argc -1){
\t\t\t(*argc)--;
\t\t}
\t}
}

'''
        argStr.append(mystr)
        argStr.append('void* runGui(void*) {\n')
        argStr.append('\tsystem("./' + projectName + '_runtime.py");\n')
        argStr.append('}\n\n')

    def parentString(self, state):
        if state.parent is None:
            return 'NULL'
        else:
            return 'state'+str(state.parent.id)

    def generateMain(self, mainStr, projectName):
        mainStr.append('int main(int argc, char* argv[]) {\n')
        mainStr.append('\tInterfaces interfaces;\n')
        mainStr.append('\ttry {\n')
        mainStr.append('\t\tinterfaces.connectProxies(argc, argv);\n')
        mainStr.append('\t} catch (const Ice::Exception& ex) {\n')
        mainStr.append('\t\tstd::cerr << ex << std::endl;\n')
        mainStr.append('\t\tinterfaces.destroyProxies();\n')
        mainStr.append('\t\treturn 1;\n')
        mainStr.append('\t} catch (const char* msg) {\n')
        mainStr.append('\t\tstd::cerr << msg << std::endl;\n')
        mainStr.append('\t\tinterfaces.destroyProxies();\n')
        mainStr.append('\t\treturn 1;\n')
        mainStr.append('\t}\n\n')

        mainStr.append('\treadArgs(&argc, argv);\n\n')

        mainStr.append('\tif (displayGui) {\n')
        mainStr.append('\t\tpthread_create(&guiThread, NULL, &runGui, NULL);\n')
        mainStr.append('\t\trunTimeGui = new RunTimeGui();\n\n')
        mainStr.append('\t}\n')

        # create state instances
        for state in self.getAllStates():
            mainStr.append('\tState* state' + str(state.id) + ' = new State' + str(state.id) + '(' +
                           str(state.id) + ', ' + str(state.initial).lower() + ', &interfaces, ' + str(state.getTimeStep()) +
                           ', ' + self.parentString(state) + ', runTimeGui);\n')
        mainStr.append('\n')

        # create transition instances
        for tran in self.getAllTransitions():
            if tran.getType() == TransitionType.CONDITIONAL:
                mainStr.append('\tTransition* tran' + str(tran.id) + ' = new Tran' + str(tran.id) +
                               '(' + str(tran.id) + ', ' + str(tran.destination.id) + ', &interfaces);\n')
            elif tran.getType() == TransitionType.TEMPORAL:
                mainStr.append('\tTransition* tran' + str(tran.id) + ' = new Tran' + str(tran.id) +
                               '(' + str(tran.id) + ', ' + str(tran.destination.id) + ', ' + str(tran.getTemporalTime()) + ');\n')

            mainStr.append('\tstate' + str(tran.origin.id) + '->addTransition(tran' + str(tran.id) + ');\n')
        mainStr.append('\n')

        for state in self.states:
            mainStr.append('\tstate' + str(state.id) + '->startThread();\n')
        mainStr.append('\n')

        for state in self.states:
            mainStr.append('\tstate' + str(state.id) + '->join();\n')
        mainStr.append('}\n')

    def generateRunTimeGui(self, guiStr):
        guiStr.append('#!/usr/bin/python\n')
        guiStr.append('# -*- coding: utf-8 -*-\n')
        guiStr.append('import sys\n')
        guiStr.append('sys.path.append("' + CMAKE_INSTALL_PREFIX + '/lib/python2.7/visualStates_py")\n\n')

        guiStr.append('from PyQt5.QtWidgets import QApplication\n')
        guiStr.append('from codegen.python.runtimegui import RunTimeGui\n\n')
        guiStr.append('gui = None\n\n')
        guiStr.append('def runGui():\n')
        guiStr.append('\tglobal gui\n')
        guiStr.append('\tapp = QApplication(sys.argv)\n')
        guiStr.append('\tgui = RunTimeGui()\n')
        guiStr.append('\tgui.activateIPC()\n\n')

        # create runtime state code
        for state in self.getAllStates():
            guiStr.append('\tgui.addState(' + str(state.id) + ', "' + state.name +
                           '", ' + str(state.initial) + ', ' + str(state.x) + ', ' + str(state.y))
            if state.parent is None:
                guiStr.append(', None)\n')
            else:
                guiStr.append(', ' + str(state.parent.id) + ')\n')
        guiStr.append('\n')

        for tran in self.getAllTransitions():
            guiStr.append('\tgui.addTransition(' + str(tran.id) + ', "' + tran.name + '", ' +
                           str(tran.origin.id) + ', ' + str(tran.destination.id) +
                           ', ' + str(tran.x) + ', ' + str(tran.y) + ')\n')
        guiStr.append('\n')

        guiStr.append('\tgui.emitLoadFromRoot()\n')
        guiStr.append('\tgui.emitActiveStateById(0)\n')
        guiStr.append('\tgui.show()\n')
        guiStr.append('\tapp.exec_()\n\n')

        guiStr.append('if __name__ == "__main__":\n')
        guiStr.append('\trunGui()\n\n')


    def generateCmake(self, cmakeStr, projectName):
        cmakeStr.append('project(')
        cmakeStr.append(projectName)
        cmakeStr.append(')\n\n')

        cmakeStr.append('cmake_minimum_required(VERSION 2.8)\n\n')

        cmakeStr.append('SET(SOURCE_FILES\n')
        cmakeStr.append('\t')
        cmakeStr.append(projectName)
        cmakeStr.append('.cpp')
        cmakeStr.append(')\n\n')

        cmakeStr.append('SET(JDEROBOT_INSTALL_PATH ' + CMAKE_INSTALL_PREFIX + ')\n')

        mystr = '''
        
SET(JDEROBOT_INCLUDE_DIR ${JDEROBOT_INSTALL_PATH}/include)
SET(VISUALSTATE_RUNTIME_INCLUDE_DIR ${JDEROBOT_INSTALL_PATH}/include/visualstates_py)

SET(JDEROBOT_LIBS_DIR ${JDEROBOT_INSTALL_PATH}/lib)
SET(VISUALSTATE_RUNTIME_LIBS_DIR ${JDEROBOT_INSTALL_PATH}/lib/visualstates_py)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
        
include_directories(
    ${JDEROBOT_INCLUDE_DIR}
    ${VISUALSTATE_RUNTIME_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

link_directories(
    ${JDEROBOT_LIBS_DIR}
    ${VISUALSTATE_RUNTIME_LIBS_DIR}
)

'''
        cmakeStr.append(mystr)
        cmakeStr.append('add_executable(')
        cmakeStr.append(projectName)
        cmakeStr.append(' ${SOURCE_FILES})\n\n')
        cmakeStr.append('target_link_libraries(\t')
        cmakeStr.append(projectName)
        cmakeStr.append('\n')

        mystr = '''visualStatesRunTime
    config
    comm
    JderobotInterfaces
    jderobotutil
    colorspacesmm
    pthread
    Ice
    IceUtil
    IceStorm
    glog
)
'''
        cmakeStr.append(mystr)

        return cmakeStr











































































