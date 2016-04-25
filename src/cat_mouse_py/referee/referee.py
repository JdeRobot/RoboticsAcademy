#!/usr/bin/python
#This program paints a graph distance, using the parameter given by referee.cfg
#VisorPainter class re-paints on a pyplot plot and updates new data.
#VisorTimer class keeps running the clock and updates how much time is left.
#Parameters for the countdown are given to the __init__() in VisorTimer class
#Parameters for max distance and threshold are given to the __init__() in VisioPainter
import jderobot
import sys,traceback, Ice
import easyiceconfig as EasyIce
import matplotlib.pyplot as plt
import numpy as np
import random
import threading
import math
from datetime import timedelta,datetime,time,date
#Install matplotlib with apt-get install python-maplotlib 
import matplotlib as mpl
#Turns off the default tooldbar
mpl.rcParams['toolbar'] = 'None'

class Pose:
	def __init__(self,argv=sys.argv):
		self.lock = threading.Lock()
		self.dist=0
		self.ic = None
		try:
			self.ic = EasyIce.initialize(sys.argv)
			self.properties = self.ic.getProperties()
	
			self.basePoseAr = self.ic.propertyToProxy("Referee.Cat.Pose3D.Proxy")
			self.poseProxy = jderobot.Pose3DPrx.checkedCast(self.basePoseAr)
			print self.poseProxy
			if not self.basePoseAr:
				raise Runtime("Cat Pose3D -> Invalid proxy")
	
			self.baseRedPoseAr = self.ic.propertyToProxy("Referee.Mouse.Pose3D.Proxy")
			self.poseRedProxy = jderobot.Pose3DPrx.checkedCast(self.baseRedPoseAr)
			print self.poseRedProxy
			if not self.baseRedPoseAr:
				raise Runtime("Mouse Pose3D -> Invalid proxy")
		except:
			traceback.print_exc()
			status = 1
			
	def update(self):
		self.lock.acquire()
		self.poseAr=self.poseProxy.getPose3DData()
		self.poseRed=self.poseRedProxy.getPose3DData()
		self.lock.release()
		return self.getDistance()
		
		
	def getDistance(self):
		v_d=pow(self.poseRed.x-self.poseAr.x,2)+pow(self.poseRed.y-self.poseAr.y,2)+pow(self.poseRed.z-self.poseAr.z,2)
		self.dist=round(abs(math.sqrt(v_d)),4)
		return self.dist
		
	def finish(self):
		if self.ic:
			#Clean up
			try:
				self.ic.destroy()
			except:
				traceback.print_exc()
				status = 1

class VisorPainter:
#Threhold is the line where points have differqent colour
	def __init__(self, threshold=7.0, max_d=20):
		self.fig, self.ax = plt.subplots()
		self.d = []
		self.t = []
		self.score=0.0		
		self.th = threshold
		self.max_dist = max_d
		self.suptitle = self.fig.suptitle('Timer is ready',fontsize=20)
		self.fig.subplots_adjust(top=0.86)
		self.score_text = self.ax.text((120.95), self.max_dist+1.5, 'Score: '+ str(self.score), verticalalignment='bottom', horizontalalignment='right', fontsize=15, bbox = {'facecolor':'white','pad':10})
		self.drawThreshold()
		self.ax.xaxis.tick_top()
		self.ax.set_xlabel('Time')
		self.ax.xaxis.set_label_position('top') 
		self.ax.set_ylabel('Distance')
		
# Sets time and distance axes.
	def setAxes(self, xaxis=120, yaxis=None):
		if (yaxis == None):
			yaxis=self.max_dist
		if (xaxis!=120):
			self.score_text.set_x((xaxis+2.95))	
		self.ax.set_xlim(0.0,xaxis)
		self.ax.set_ylim(yaxis,0)

# Draws the threshold line
	def drawThreshold(self):
		plt.axhline(y=self.th)

# Draws points. Green ones add 1 to score.
# Not in use.
	def drawPoint(self,t_list,d_list):
		if d<=self.th:	
			self.score+=1
			plt.plot([t],[d], 'go', animated=True)
		else:
			plt.plot([t],[d], 'ro', animated=True)
	
# Decides if it's a Green or Red line. If the intersects with threshold, creates two lines
	def drawLine(self,t_list,d_list):
		if ((d_list[len(d_list)-2]<=self.th) and (d_list[len(d_list)-1]<=self.th)):
			self.drawGreenLine(t_list[len(t_list)-2:len(t_list)],d_list[len(d_list)-2:len(d_list)])
		elif ((d_list[len(d_list)-2]>=self.th) and (d_list[len(d_list)-1]>=self.th)):
			self.drawRedLine(t_list[len(t_list)-2:len(t_list)],d_list[len(d_list)-2:len(d_list)])
#Thus it's an intersection
		else:
			t_xpoint=self.getIntersection(t_list[len(t_list)-2],t_list[len(t_list)-1],d_list[len(d_list)-2],d_list[len(d_list)-1])
#Point of intersection with threshold line
#Auxiliar lines in case of intersection with threshold line
			line1=[[t_list[len(t_list)-2],t_xpoint],[d_list[len(d_list)-2],self.th]]
			line2=[[t_xpoint,t_list[len(t_list)-1]],[self.th,d_list[len(d_list)-1]]]
			self.drawLine(line1[0],line1[1])
			self.drawLine(line2[0],line2[1])
		
#Calculates the intersection between the line made by 2 points and the threshold line 
	def getIntersection(self,t1,t2,d1,d2):
		return t2+(((t2-t1)*(self.th-d2))/(d2-d1))
						
	def drawGreenLine(self,t_line,d_line):
		self.score+=(t_line[1]-t_line[0])
		plt.plot(t_line,d_line,'g-')

	def drawRedLine(self,t_line,d_line):
		plt.plot(t_line,d_line,'r-')
		
# Updates score
	def update_score(self):
		if self.score <= vt.delta_t.total_seconds():
			self.score_text.set_text(str('Score: %.2f secs' % self.score))
		else:
			self.score_text.set_text('Score: ' +  str(vt.delta_t.total_seconds())+ ' secs')
		
		
#Updates timer
	def update_title(self):
		#self.update_score()
		if vt.timeLeft() <= vt.zero_t:
			vt.stopClkTimer()
			self.suptitle.set_text(
str(vt.zero_t.total_seconds()))
			self.ax.figure.canvas.draw()		
		else:
			self.suptitle.set_text(str(vt.timeLeft())[:-4])
			self.ax.figure.canvas.draw()

#Updates data for drawing into the graph
#The first data belongs to 0.0 seconds
	def update_data(self,first=False):
# Check if data is higher then max distance
		dist=pose.update()
		if first:
			self.t.insert(len(self.t),0.0)
		else:	
			self.t.insert(len(self.t),(vt.delta_t-vt.diff).total_seconds())
		if dist > self.max_dist :
			self.d.insert(len(self.d),self.max_dist)
		else:
			self.d.insert(len(self.d),dist)
#		self.drawPoint(self.t[len(self.t)-1],self.d[len(self.d)-1])
  		if len(self.t)>=2 and len(self.d)>=2:
			self.drawLine(self.t,self.d)
			self.update_score()
		if vt.timeLeft() <= vt.zero_t:
			vt.stopDataTimer()
			self.update_score()
			self.ax.figure.canvas.draw()
			self.fig.savefig('Result_'+str(datetime.now())+'.png', bbox_inches='tight')
			
#https://github.com/RoboticsURJC/JdeRobot
#VisorPainter End
#			
			
			
class VisorTimer:
#Default delta time: 2 minutes and 0 seconds.
#Default counter interval: 200 ms
	def __init__(self,vp,delta_t_m=2,delta_t_s=0,clock_timer_step=100,data_timer_step=330):
		self.delta_t = timedelta(minutes=delta_t_m,seconds=delta_t_s)
		self.zero_t = timedelta(minutes=0,seconds=0,milliseconds=0)
		self.final_t = datetime.now()+self.delta_t
		self.diff = self.final_t-datetime.now()
		vp.setAxes(xaxis=self.delta_t.seconds)
# Creates a new clock_timer object. 

		self.clock_timer = vp.fig.canvas.new_timer(interval=clock_timer_step)
		self.data_timer = vp.fig.canvas.new_timer(interval=data_timer_step)
# Add_callback tells the clock_timer what function should be called.
		self.clock_timer.add_callback(vp.update_title)
		self.data_timer.add_callback(vp.update_data)	
			
	def startTimer(self):
		self.clock_timer.start()
		vp.update_data(first=True)
		self.data_timer.start()
		
		
	def stopClkTimer(self,):
		self.clock_timer.remove_callback(vp.update_title)
		self.clock_timer.stop()
		
	def stopDataTimer(self):
		self.data_timer.remove_callback(vp.update_data)
		self.data_timer.stop()
	
		
	def timeLeft(self):
		self.diff=self.final_t-datetime.now()
		return self.diff
#
#VisorTimer End
#

# Main

status = 0
	
try:
	pose = Pose(sys.argv)
	pose.update()
	vp = VisorPainter()
	vt = VisorTimer(vp)
	vp.suptitle.set_text(str(vt.delta_t))
	vt.startTimer()
	plt.show()
	pose.finish()
	
except:
	traceback.print_exc()
	status = 1

			
sys.exit(status)
