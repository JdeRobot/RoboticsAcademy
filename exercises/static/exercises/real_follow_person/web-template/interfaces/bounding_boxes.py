import rclpy
from rclpy.node import Node
import threading
from darknet_ros_msgs.msg import BoundingBoxes
    
class BoundingBoxData:

	def __init__(self):
		self.probability = 0
		self.xmin = 0
		self.ymin = 0
		self.xmax = 0
		self.ymax = 0
		self.id = 0
		self.class_id = ""


def BoundingBox2BoundingBoxData(bbox):
	bounding_box = BoundingBoxData()
	bounding_box.probability = bbox.probability
	bounding_box.xmin = bbox.xmin
	bounding_box.ymin = bbox.ymin
	bounding_box.xmax = bbox.xmax
	bounding_box.ymax = bbox.ymax
	bounding_box.id = bbox.id
	bounding_box.class_id = bbox.class_id
	
	return bounding_box


class BoundingBoxesData:
	
	def __init__(self):
		self.timeStamp = 0
		self.bounding_boxes = []
	
	def __str__(self):
		s = "TimeStamp: " + str(self.timeStamp) + "\nBoundingBoxes: " + str(self.bounding_boxes) + "\n"
		return s
		

def BoundingBoxes2BoundingBoxesData(bboxes):
	
	bounding_boxes = BoundingBoxesData()
	bounding_boxes.bounding_boxes = bboxes.bounding_boxes
	bounding_boxes.timeStamp = bboxes.header.stamp.sec + (bboxes.header.stamp.nanosec *1e-9)
	return bounding_boxes

class ListenerBoundingBoxes(Node):
	'''
		Bounding Boxes Subscriber
	'''
	def __init__(self, topic):
		super().__init__("bounding_boxes_subscriber_node")
		self.topic = topic
		self.data = BoundingBoxesData()
		self.sub = None
		self.lock = threading.Lock()
		self.start()
	
	def __callback(self, bboxes):
		bounding_boxes = BoundingBoxes2BoundingBoxesData(bboxes)
		
		self.lock.acquire()
		self.data = bounding_boxes
		self.lock.release()
		
	def start(self):
		self.sub = self.create_subscription(BoundingBoxes, self.topic, self.__callback, 10)
	
	def stop(self):
		self.sub.unregister()
	
	def getBoundingBoxes(self):
		self.lock.acquire()
		bounding_boxes = self.data
		self.lock.release()
		
		return bounding_boxes
	
		
