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
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPainter, QPixmap
from PyQt5.QtWidgets import QMainWindow, QDockWidget, QTreeView, QGraphicsView, \
    QWidget, QLabel, QVBoxLayout, QPushButton, QGraphicsItem, \
    QGraphicsScene
from gui.treemodel import TreeModel
from gui.state import State
from gui.transition import Transition
from gui.cmakevars import CMAKE_INSTALL_PREFIX

from threading import Thread
import time
import sysv_ipc

class RunTimeGui(QMainWindow):

    activeStateChanged = pyqtSignal(int)
    runningStateChanged = pyqtSignal(int)
    loadFromRoot = pyqtSignal(int)

    def __init__(self, parent=None):
        super(QMainWindow, self).__init__()

        self.setWindowTitle("VisualStates RunTime GUI")

        self.rootState = None

        # create status bar
        self.statusBar()

        self.createTreeView()
        self.createStateCanvas()

        self.setGeometry(0, 0, 800, 600)
        self.show()

        self.states = {}
        self.transitions = {}

        self.activeState = None

        self.activeStateChanged.connect(self.activeStateChangedHandle)
        self.runningStateChanged.connect(self.runningStateChangedHandle)
        self.loadFromRoot.connect(self.loadFromRootHandle)

        self.memory = None
        self.ipcThread = None
        self.semaphore = None

    def createTreeView(self):
        dockWidget = QDockWidget()
        dockWidget.setAllowedAreas(Qt.LeftDockWidgetArea)
        dockWidget.setFeatures(QDockWidget.NoDockWidgetFeatures)
        dockWidget.setTitleBarWidget(QWidget())
        self.treeView = QTreeView()
        self.treeView.clicked.connect(self.treeItemClicked)
        self.treeModel = TreeModel()
        self.treeView.setModel(self.treeModel)

        self.logo = QLabel()
        logoPixmap = QPixmap(CMAKE_INSTALL_PREFIX + '/share/jderobot/resources/jderobot.png')
        self.logo.setPixmap(logoPixmap)

        self.upButton = QPushButton()
        self.upButton.setText('Up')
        self.upButton.clicked.connect(self.upButtonClicked)

        leftContainer = QWidget()
        leftLayout = QVBoxLayout()
        leftLayout.addWidget(self.treeView)
        leftLayout.addWidget(self.upButton)
        leftLayout.addWidget(self.logo)
        leftContainer.setLayout(leftLayout)

        dockWidget.setWidget(leftContainer)
        self.addDockWidget(Qt.LeftDockWidgetArea, dockWidget)

    def createStateCanvas(self):
        self.stateCanvas = QGraphicsView()
        self.scene = QGraphicsScene()
        self.scene.setSceneRect(0, 0, 2000, 2000)

        self.setCentralWidget(self.stateCanvas)
        self.stateCanvas.setScene(self.scene)
        self.stateCanvas.setRenderHint(QPainter.Antialiasing)


    def addState(self, id, name, initial, x, y, parentId):
        if parentId is not None:
            self.states[id] = State(id, name, initial, self.states[parentId])
            self.states[parentId].addChild(self.states[id])
            parentItem = self.treeModel.getByDataId(parentId)
            # print('parent:' + str(parentItem))
        else:
            self.states[id] = State(id, name, initial, None)
        if id == 0:
            self.rootState = self.states[id]

        self.states[id].setPos(x, y)

    def addTransition(self, id, name, originId, destinationId, x, y):
        self.transitions[id] = Transition(id, name, self.states[originId], self.states[destinationId])
        self.transitions[id].setPos(x, y)

    def emitRunningStateById(self, id):
        # print('emit running state:' + str(id))
        self.runningStateChanged.emit(id)

    def runningStateChangedHandle(self, id):
        # print('running state:' + str(id))
        if id not in self.states:
            return

        runningState = self.states[id]

        parentId = None
        if runningState.parent is not None:
            for child in runningState.parent.getChildren():
                child.setRunning(False)

            runningState.setRunning(True)
            parentId = runningState.parent.id

        self.treeModel.setAllBackgroundByParentId(Qt.white, parentId)
        self.treeModel.setBackgroundById(runningState.id, Qt.green)

    def emitActiveStateById(self, id):
        self.activeStateChanged.emit(id)

    def activeStateChangedHandle(self, id):
        if self.activeState is not None:
            for child in self.activeState.getChildren():
                child.resetGraphicsItem()
                for tran in child.getOriginTransitions():
                    tran.resetGraphicsItem()

        self.activeState = self.states[id]
        # print('set active state:' + str(id))
        self.scene.clear()
        for childState in self.activeState.getChildren():
            # print('add child to scene:' + str(childState.id))
            qitem = childState.getGraphicsItem()
            qitem.setAcceptHoverEvents(False)
            qitem.setFlag(QGraphicsItem.ItemIsMovable, False)
            qitem.doubleClicked.connect(self.stateDoubleClicked)
            self.setAcceptDrops(False)
            self.scene.addItem(qitem)
            for tran in childState.getOriginTransitions():
                # print('add transition:' + str(tran.id))
                qitem = tran.getGraphicsItem()
                qitem.disableInteraction()
                self.scene.addItem(qitem)

    def emitLoadFromRoot(self):
        self.loadFromRoot.emit(0)

    def loadFromRootHandle(self, id):
        self.treeModel.loadFromRoot(self.states[id])

    def stateDoubleClicked(self, stateItem):
        if len(stateItem.stateData.getChildren()) > 0:
            self.emitActiveStateById(stateItem.stateData.id)

    def upButtonClicked(self):
        if self.activeState is not None:
            if self.activeState.parent is not None:
                self.emitActiveStateById(self.activeState.parent.id)

    def getStateById(self,state, id):
        if state.id == id:
            return state
        else:
            result = None
            for child in state.getChildren():
                result = self.getStateById(child, id)
                if result is not None:
                    return result
            return result

    def treeItemClicked(self, index):
        # print('clicked item.id:' + str(index.internalPointer().id))
        pass

    def getStateList(self, state, stateList):
        if len(state.getChildren()) > 0:
            stateList.append(state)

        for s in state.getChildren():
            self.getStateList(s, stateList)

    def loopIPC(self):
        while True:
            msg = self.getIPCMessage()
            if msg is not None:
                if len(msg) > 0:
                    msg = msg.strip()
                    methodName = msg.split(' ')[0]
                    id = int(msg.split(' ')[1])
                    if methodName == 'emitRunningStateById':
                        self.emitRunningStateById(id)
                    else:
                        print('unknown method name')

            # time.sleep(1.0/1000)

    def activateIPC(self):
        try:
            self.memory = sysv_ipc.SharedMemory(123456, flags=sysv_ipc.IPC_CREAT, size = 1024)
        except:
            print('cannot create or open share mem with id 123456')
            return

        # create sempahore
        try:
            self.semaphore = sysv_ipc.Semaphore(9, flags=sysv_ipc.IPC_CREAT, initial_value=0)
        except:
            print('cannot create or open semaphore with id 9')
            self.memory.remove()
            return

        self.ipcThread = Thread(target=self.loopIPC)
        self.ipcThread.start()

    def getIPCMessage(self):
        self.semaphore.acquire()
        data = self.memory.read().decode()
        return data





