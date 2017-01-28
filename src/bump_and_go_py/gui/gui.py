#!/usr/bin/env python3
	
from PyQt5.QtCore import QPoint, QRect, QSize, Qt, QPointF, QRectF
from PyQt5.QtGui import (QBrush, QConicalGradient, QLinearGradient, QPainter,
		QPainterPath, QPalette, QPen, QPixmap, QPolygon, QRadialGradient, QColor, 
		QTransform, QPolygonF, QKeySequence, QIcon)
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QGridLayout,
		QLabel, QSpinBox, QWidget, QPushButton, QSpacerItem, QSizePolicy )
  
import basicdrawing_rc

from gui.machine import *
from gui.state import *
from gui.transition import *
import math, time
  
  
class RenderArea(QWidget):
	  
	Line, Points, Polyline, Polygon, Rect, RoundedRect, Ellipse, Arc, Chord, \
			Pie, Path, Text, Pixmap = range(13)
  
	def __init__(self, machine, parent=None):
		super(RenderArea, self).__init__(parent)
  
		self.pen = QPen()
		self.brush = QBrush()
		
		self.antialiased = True
  
		self.setBackgroundRole(QPalette.Base)
		self.setAutoFillBackground(True)
		
		self.machine = machine

		self.dist_radius = min(self.width() / 4, self.height() / 4) * 1.5
		self.dist_center = QPoint(self.width() / 2.0, self.height() / 2.0)
		self.n_states = len(self.machine.getStates())

		self.state_radius = self.funcc(self.dist_radius, self.n_states)
		self.active = -1
		self.t_active = -1

		self.pts = []


		#Brushes
		linearGradient = QLinearGradient(-self.state_radius, -self.state_radius, self.state_radius, self.state_radius)
		linearGradient.setColorAt(0.0, Qt.darkGreen)
		linearGradient.setColorAt(0.7, Qt.green)
		linearGradient.setColorAt(0.3, Qt.green)
		linearGradient.setColorAt(1.0, Qt.white)
		self.greenGradientBrush = QBrush(linearGradient)

		linearGradient = QLinearGradient(-self.state_radius, -self.state_radius, self.state_radius, self.state_radius)
		linearGradient.setColorAt(0.0, Qt.darkGray)
		linearGradient.setColorAt(0.7, Qt.lightGray)
		linearGradient.setColorAt(0.3, Qt.gray)
		linearGradient.setColorAt(1.0, Qt.white)
		self.grayGradientBrush = QBrush(linearGradient)
		self.setBrush(self.grayGradientBrush)
		#end brushes

	def funcc(self, r, n):
		return int((r*0.05) / (n*0.04))

	def minimumSizeHint(self):
		return QSize(100, 100)
  
	def sizeHint(self):
		return QSize(400, 200)
  
	def setPen(self, pen):
		self.pen = pen
		self.update()
  
	def setBrush(self, brush):
		self.brush = brush
		self.update()
 
	def resizeEvent(self, event):
		self.dist_center = QPoint(self.width() / 2.0, self.height() / 2.0)
		self.dist_radius = min(self.width() / 4, self.height() / 4) * 1.5
		self.state_radius = self.funcc(self.dist_radius, self.n_states)
		self.update()

	def poly(self, pts):
		return QPolygonF(map(lambda p: QPointF(*p), pts))

	def wave(self):
		
		t = self.machine.getActiveTransition()
		if t != None:

			init = QPoint(t.getOrig()[0], t.getOrig()[1])
			end = QPoint(t.getDest()[0], t.getDest()[1])
			angle = t.getAngle()
			#print('processing transition', t.id, t.orig, t.dest)
			while t.isActive():
			#for i in range(3):	#for testing
				self.pts = [[init.x(), init.y()], [end.x(), end.y()]]
				if self.pts[0][0] <= self.pts[1][0] and self.pts[0][1] >= self.pts[1][1]:
					while self.pts[0][0] <= self.pts[1][0] and self.pts[0][1] >= self.pts[1][1]:
						self.sumPoint(angle)
				elif self.pts[0][0] <= self.pts[1][0] and self.pts[0][1] <= self.pts[1][1]:
					while self.pts[0][0] <= self.pts[1][0] and self.pts[0][1] <= self.pts[1][1]:
						self.sumPoint(angle)
				elif self.pts[0][0] >= self.pts[1][0] and self.pts[0][1] >= self.pts[1][1]:
					while self.pts[0][0] >= self.pts[1][0] and self.pts[0][1] >= self.pts[1][1]:
						self.sumPoint(angle)
				elif self.pts[0][0] >= self.pts[1][0] and self.pts[0][1] <= self.pts[1][1]:
					while self.pts[0][0] >= self.pts[1][0] and self.pts[0][1] <= self.pts[1][1]:
						self.sumPoint(angle)
						

	def sumPoint(self, angle):
		self.pts[0][0] += (self.state_radius * math.cos(angle) * 0.1)
		self.pts[0][1] -= (self.state_radius * math.sin(angle) * 0.1)
		self.update()
		QApplication.processEvents()
		time.sleep(0.025)

	def paintEvent(self, event):
		  
		painter = QPainter(self)
		painter.setPen(self.pen)
		painter.setBrush(self.brush)
		if self.antialiased:
			painter.setRenderHint(QPainter.Antialiasing)

		angle_step = 360 / self.n_states

		painter.save()	#Save_1. Save the state of the system (push matrix)
		painter.translate(self.dist_center.x(), self.dist_center.y())		# go to the center of the render area
		painter.rotate(-180)	#to start painting from the left side of the circle (clockwise)

		#center of the circumference where through we are going to paint our states 
		x = self.dist_radius * math.cos(0)	
		y = self.dist_radius * math.sin(0)


		for h in range(self.n_states):

			rot = angle_step * h 	# each state is equidistant from the others. We paint them in circles

			painter.save()			#Save_2
			painter.rotate(rot)		#now our system is pointing to the next state to be drawn
			painter.translate(x,y)	#now our origin is in the center of the next state to be drawn

			#if the state is active, fill it green
			if self.machine.getState(h).isActive():	
				painter.setBrush(self.greenGradientBrush)			
			painter.drawEllipse(QPoint(0,0), self.state_radius, self.state_radius)	#draw the new state

			#global position of transformed coordinates (before any transformation, origin at top-left corner)
			gx = painter.worldTransform().map(QPoint(0,0)).x()
			gy = painter.worldTransform().map(QPoint(0,0)).y()
			self.machine.getState(h).setPos(gx, gy)		#store the center of the state without any transformation applied

			# text transformation. Our origin is still in the center of the current state
			painter.save()			#Save_3
			painter.rotate(180)		#making the text go vertical
			painter.rotate(-rot)	#undoing the rotation made for painting the state. No the text is horizontal
			font = painter.font();
			font.setPixelSize(self.state_radius*.4);
			painter.setFont(font);
			rect = QRect(-self.state_radius, -self.state_radius, self.state_radius*2, self.state_radius*2)
			painter.drawText(rect, Qt.AlignCenter, self.machine.getState(h).getName());
			painter.restore()	#Restore_3
			#end text transformation

			painter.restore()	#Restore_2
			
		painter.restore()	#Restore_1. Restore the state of the system (pop matrix)

		
		#drawing transitions. Line between states
		painter.save()	# Save_4
		pptv = QTransform()		#Define a new transformation. Needed to rotate the system along other axis than Z
		pptv.translate(0, self.height())	#We are now at the bottom-left corner of the screen
		pptv.rotate(-180, Qt.XAxis)			#Rotate along the X-axis so now we are in a typical cartesian system.
		painter.setTransform(pptv)			#Apply the transformation
		states = self.machine.getStates()
		for state in states:
			transitions = state.getTransitions()
			for transition in transitions:
				#get the center of the origin and destination states in our current system state
				orig = QPoint(state.getPos()[0], state.getPos()[1])
				end = QPoint(self.machine.getState(transition.getStateEnd()).getPos()[0], self.machine.getState(transition.getStateEnd()).getPos()[1])
				# get those coordinates without transformation
				orig2 = QPoint(painter.worldTransform().map(orig))
				end2 = QPoint(painter.worldTransform().map(end))

				#get the angle between states centers and the horizon
				angle = math.atan2(end2.y() - orig2.y(), end2.x() - orig2.x())

				#get the coordinates of the starting point of the transition (it starts in the bound of the state, not in the center)
				newX = self.state_radius * math.cos(angle) + orig2.x()
				newY = self.state_radius * math.sin(angle) + orig2.y()
				#now the transition starts at the border, not in the center
				orig2.setX(newX)
				orig2.setY(newY)

				#same for the destination state
				angle2 = math.atan2(orig2.y() - end2.y(), orig2.x() - end2.x())
				newX2 = self.state_radius * math.cos(angle2) + end2.x()
				newY2 = self.state_radius * math.sin(angle2) + end2.y()
				end2.setX(newX2)
				end2.setY(newY2)

				#draw the line between the origin and destination states
				painter.drawLine(orig2, end2)
				#get the start and the end of the transition untransformed
				init = QPoint(painter.worldTransform().map(orig2))
				end = QPoint(painter.worldTransform().map(end2))
				#store that info
				transition.setOrig(init.x(), init.y())
				transition.setDest(end.x(), end.y())	
				transition.setAngle(angle)
		painter.restore() #Restore_4


		#Appliying style to the transitions
		painter.setPen(QPen(QColor(Qt.gray), 3))
		for state in self.machine.getStates():
			for transition in state.getTransitions():
				#get the start and end coordinates of the transition
				i = QPoint(transition.getOrig()[0], transition.getOrig()[1])
				o = QPoint(transition.getDest()[0], transition.getDest()[1])			
				painter.drawPolyline(i, o)

				#Drawing the arrow at the end of the transition
				painter.save()	#Save_5
				painter.setPen(QPen(QColor(Qt.gray), 2))
				painter.translate(transition.getDest()[0],transition.getDest()[1])	#Go to the end of the transition
				painter.rotate(90 - transition.getAngle()*180/math.pi)		#Rotate to point in the direction of the transition

				#coordinates of the arrow (triangle)
				a = QPoint(0,0)
				b = QPoint(-5,10)
				c = QPoint(5,10)

				#coordinates of the arrow untransformed
				a1 = painter.worldTransform().map(a)
				b1 = painter.worldTransform().map(b)
				c1 = painter.worldTransform().map(c)

				#Drawin the actual arrow
				pointer = QPolygon([a,b,c])
				painter.drawPolygon(pointer)
				painter.restore()	#Restore_5

				#For the animation of the transition
				painter.save()	#Save_6
				if transition.isActive():	#if the current transition is the active one the wave function will be running, so it's updating the canvas

					painter.setPen(QPen(QColor(Qt.green), 3))
					painter.drawPolyline(i,o)
					
					painter.setPen(QPen(QColor(Qt.gray), 3))
					painter.drawPolyline(self.poly(self.pts))

					#Draw the arrow in the active state (red arrow)
					painter.setBrush(QBrush(QColor(255, 0, 0)))
					painter.setPen(QPen(QColor(Qt.red), 2))
					pointer = QPolygon([a1,b1,c1])
					painter.drawPolygon(pointer)
					
					#Ball that follows the line animation
					for x, y in self.pts:
						painter.drawEllipse(QRectF(self.pts[0][0] - 4, self.pts[0][1] - 4, 8, 8))
				painter.restore()	#Restore_6

				#Painting the text of the transition
				painter.save()	#Save_7
				pptv = QTransform()
				painter.setPen(QPen(QColor(Qt.black), 3))
				#get the middle point of the transition
				middleX = (transition.getOrig()[0] + transition.getDest()[0]) /2	
				middleY = (transition.getOrig()[1] + transition.getDest()[1]) /2
				pptv.translate(middleX, middleY)	#translate to that point
				painter.setTransform(pptv)			#apply the transformation
				font = painter.font();
				font.setPixelSize(self.state_radius*.2);
				painter.setFont(font);
				rect = QRect(-self.state_radius, -self.state_radius, self.state_radius*2, self.state_radius*2)
				name = str(transition.getId())+ '. ' + transition.getName()
				painter.drawText(rect, Qt.AlignCenter, name)
				painter.restore()	#Restore_7


  
		#paint the actual canvas
		painter.setPen(self.palette().dark().color())
		painter.setBrush(Qt.NoBrush)
		painter.drawRect(QRect(0, 0, self.width() - 1, self.height() - 1))


	def start(self,n):
		'''print('active', n)
		self.active = n%3
		self.deactivateAll()
		self.machine.setStateActive(self.active, True)
		#time.sleep(5)
		self.t_active = n%3
		self.wave()

		for i in range(self.n_states):
			print('active', i)
			self.active = i
			self.deactivateAll()
			self.machine.setStateActive(self.active, True)
			self.update()
			QApplication.processEvents()
			time.sleep(2)

			self.t_active = i
			self.deactivateAll()
			self.machine.setTransitionActive  (self.t_active, True)
			for i in self.machine.getTransitions():
				print (i.isActive())
			self.update()
			QApplication.processEvents()
			self.wave()'''

	def stop(self):
		None
		#self.machine.deactivateAll()


IdRole = Qt.UserRole
  
class Window(QWidget):
	def __init__(self, machine, algorithm):
		super(Window, self).__init__()
  
		self.renderArea = RenderArea(machine)

		self.algorithm = algorithm
  
		mainLayout = QGridLayout()
		mainLayout.addWidget(self.renderArea, 0, 0, 1, 4)
		mainLayout.setRowMinimumHeight(1, 6)

		self.startButton = QPushButton()
		self.startButton.setText('&Play')
		self.startButton.setIcon(QIcon('./resources/play.png'))
		#self.startButton.setShortcut(QKeySequence(Qt.Key_P))    #same effect with & in button constructor

		self.stopButton = QPushButton()
		self.stopButton.setText('&Stop')
		self.stopButton.setIcon(QIcon('./resources/stop.png'))

		self.infoButton = QPushButton()
		self.infoButton.setText('&Info')

		mainLayout.addWidget(self.startButton, 1, 0)
		mainLayout.addWidget(self.stopButton, 1, 1)
		mainLayout.addWidget(self.infoButton, 1, 3)


		self.startButton.pressed.connect(self.startPressed)
		self.stopButton.pressed.connect(self.stopPressed)
		self.infoButton.pressed.connect(self.infoPressed)

		self.setMinimumSize(QSize(640,480))
				
		self.setLayout(mainLayout)

		self.setWindowTitle("Basic Drawing")
		self.cont = 0

	def startPressed(self):
		self.algorithm.play()

	def stopPressed(self):
		self.renderArea.stop()
		self.algorithm.stop()

	def infoPressed(self):
		None	 

	def update(self):
		self.renderArea.update()
		self.renderArea.wave()


	def closeEvent(self, event):
		self.algorithm.kill()
		event.accept()
	
if __name__ == '__main__':
  
	import sys
  
	machine = Machine(3)
	machine.setStateName(0, 'Forward') 
	machine.setStateName(1, 'Backward')
	machine.setStateName(2, 'Turn')
	machine.addTransition(0, 1,'close')
	machine.addTransition(1, 2,'time')
	machine.addTransition(2, 0,'time2')
	

	for i in machine.getTransitions():
		print(i.id, i.orig, i.dest)
	
	app = QApplication(sys.argv)
	window = Window(machine)
	window.show()
	sys.exit(app.exec_())
  
