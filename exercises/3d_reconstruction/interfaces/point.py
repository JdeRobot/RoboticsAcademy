import rospy
from std_msgs.msg import Float64MultiArray as Float
import config
import threading
from math import pi as PI
from .threadPublisher import ThreadPublisher

class ListenerPoint:
	def __init__(self, topic):
		self.topic = topic
		self.data = Float()
		self.sub = None
		self.lock = threading.Lock()
		
		self.start()
		
	def __callback(self, point_data):
		self.lock.acquire()
		self.data = point_data.data
		self.lock.release()
		
	def stop(self):
		self.sub.unregister()
		
	def start(self):
		self.sub = rospy.Subscriber(self.topic, Float, self.__callback)
		
	def getPoint(self):
		self.lock.acquire()
		point = self.data
		self.lock.release()
		
		return point
		
	def hasproxy(self):
		return hasattr(self, "sub") and self.sub
		
		
class PublisherPoint:
	def __init__(self, topic):
		self.topic = topic
		self.point = Float()
		self.pub = rospy.Publisher(self.topic, Float, queue_size=10)
		rospy.init_node("PlotPoint")
		self.lock = threading.Lock()
		
		self.kill_event = threading.Event()
		self.thread = ThreadPublisher(self, self.kill_event)
		
		self.thread.daemon = True
		self.start()
		
	def publish(self):
		self.pub.publish(self.point)
		
	def stop(self):
		self.kill_event.set()
		self.pub.unregister()
		
	def start(self):
		self.kill_event.clear()
		self.thread.start()
		
	def plotPoint(self, position, color):
		self.lock.acquire()
		self.point.data = position + color
		rospy.sleep(1)
		self.lock.release()
		
		
