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
from gui.guitransition import TransitionGraphicsItem
from PyQt5.QtCore import QPointF

class Transition:
    def __init__(self, id, name, origin=None, dest=None):
        self.id = id
        self.name = name
        self.transitionType = TransitionType.TEMPORAL
        self.code = ''
        self.temporalTime = 0
        self.condition = ''
        self.x = 0
        self.y = 0
        self.isPosChanged = False

        self.origin = None
        self.destination = None

        # set transitions on the state if origin and dest are not None
        if origin is not None:
            self.origin = origin
            self.origin.addOriginTransition(self)
        if dest is not None:
            self.destination = dest
            self.destination.addDestTransition(self)

        self.setPosFromOriginAndDestination()

        self.graphicsItem = None

    def setPos(self, x, y):
        self.x = x
        self.y = y
        self.isPosChanged = True

    def addOriginState(self, origin):
        if origin != self.origin:
            self.origin = origin
            self.origin.addOriginTransition(self)

    def addDestinationState(self, dest):
        if dest != self.destination:
            self.destination = dest
            self.destination.addDestTransition(self)

    def getGraphicsItem(self):
        if self.graphicsItem is None:
            self.graphicsItem = TransitionGraphicsItem(self)
            self.graphicsItem.posChanged.connect(self.posChanged)

        if self.isPosChanged:
            self.graphicsItem.updateMiddlePoints(QPointF(self.x, self.y))
            self.graphicsItem.createMiddleHandle()
            self.isPosChanged = False

        return self.graphicsItem

    def resetGraphicsItem(self):
        self.graphicsItem = None
        self.isPosChanged = True

    def getTemporalTime(self):
        return self.temporalTime

    def setTemporalTime(self, time):
        self.temporalTime = int(time)

    def getCondition(self):
        return self.condition

    def setCondition(self, cond):
        self.condition = cond

    def getType(self):
        return self.transitionType

    def setType(self, type):
        if type == TransitionType.TEMPORAL or type == TransitionType.CONDITIONAL:
            self.transitionType = type

    def getCode(self):
        return self.code

    def setCode(self, code):
        self.code = code

    def posChanged(self, tranItem):
        self.isPosChanged = True
        self.x = tranItem.midPointX
        self.y = tranItem.midPointY

    def setPosFromOriginAndDestination(self):
        if self.origin is not None and self.destination is not None:
            self.x = (self.origin.x + self.destination.x)/2
            self.y = (self.origin.y + self.destination.y)/2

    def createElement(self, doc):
        tranElement = doc.createElement('transition')
        tranElement.setAttribute('id', str(self.id))
        typeElement = doc.createElement('type')
        typeElement.appendChild(doc.createTextNode(str(self.getType())))
        tranElement.appendChild(typeElement)
        if self.getType() == TransitionType.CONDITIONAL:
            condElement = doc.createElement('condition')
            condElement.appendChild(doc.createTextNode(self.getCondition()))
            tranElement.appendChild(condElement)
        elif self.getType() == TransitionType.TEMPORAL:
            timeElement = doc.createElement('time')
            timeElement.appendChild(doc.createTextNode(str(self.getTemporalTime())))
            tranElement.appendChild(timeElement)
        tposxElement = doc.createElement('posx')
        tposxElement.appendChild(doc.createTextNode(str(self.x)))
        tranElement.appendChild(tposxElement)
        tposyElement = doc.createElement('posy')
        tposyElement.appendChild(doc.createTextNode(str(self.y)))
        tranElement.appendChild(tposyElement)
        nameElement = doc.createElement('name')
        nameElement.appendChild(doc.createTextNode(self.name))
        tranElement.appendChild(nameElement)
        originElement = doc.createElement('originid')
        originElement.appendChild(doc.createTextNode(str(self.origin.id)))
        tranElement.appendChild(originElement)
        destinElement = doc.createElement('destinationid')
        destinElement.appendChild(doc.createTextNode(str(self.destination.id)))
        tranElement.appendChild(destinElement)
        codeElement = doc.createElement('code')
        codeElement.appendChild(doc.createTextNode(self.getCode()))
        tranElement.appendChild(codeElement)
        return tranElement

    def parse(self, transitionElement, statesById):
        for (name, value) in transitionElement.attributes.items():
            if name == 'id':
                self.id = int(value)
        self.transitionType = int(transitionElement.getElementsByTagName('type')[0].childNodes[0].nodeValue)
        if self.transitionType == TransitionType.TEMPORAL:
            self.setTemporalTime(int(transitionElement.getElementsByTagName('time')[0].childNodes[0].nodeValue))
        elif self.transitionType == TransitionType.CONDITIONAL:
            self.setCondition(transitionElement.getElementsByTagName('condition')[0].childNodes[0].nodeValue)
        self.name = transitionElement.getElementsByTagName('name')[0].childNodes[0].nodeValue
        self.x = float(transitionElement.getElementsByTagName('posx')[0].childNodes[0].nodeValue)
        self.y = float(transitionElement.getElementsByTagName('posy')[0].childNodes[0].nodeValue)
        # parse optinal code tag
        if len(transitionElement.getElementsByTagName('code')[0].childNodes) > 0:
            self.setCode(transitionElement.getElementsByTagName('code')[0].childNodes[0].nodeValue)
        originId = int(transitionElement.getElementsByTagName('originid')[0].childNodes[0].nodeValue)
        self.addOriginState(statesById[originId])
        destinationId = int(transitionElement.getElementsByTagName('destinationid')[0].childNodes[0].nodeValue)
        self.addDestinationState(statesById[destinationId])
        self.isPosChanged = True


