from interfaces.motors import PublisherMotors
from interfaces.laser import ListenerLaser
from interfaces.camera import ListenerCamera
from interfaces.pose3d import ListenerPose3d
from interfaces.ssd_detection import NeuralNetwork, BoundingBox
from coco_labels import LABEL_MAP
from rclpy.executors import MultiThreadedExecutor
from datetime import datetime
import rclpy
import time
import threading

    
class HAL:

    def __init__(self):
    	# init node
    	rclpy.init()
    	
    	self.motors = PublisherMotors("/commands/velocity", 4, 0.3)
    	self.laser = ListenerLaser("/scan")
    	self.camera = ListenerCamera("/image_raw")
    	self.odometry = ListenerPose3d("/odom")
    	
    	self.listener_executor = MultiThreadedExecutor(num_threads=4)
    	self.listener_executor.add_node(self.laser)
    	self.listener_executor.add_node(self.odometry)
    	self.listener_executor.add_node(self.camera)
    	
    	self.net = NeuralNetwork()
    	
    	# Update thread
    	self.thread = ThreadHAL(self.listener_executor)
    
    def start_thread(self):
    	self.thread.start()
    	
    def setV(self, velocity):
    	self.motors.sendV(velocity)
    
    def setW(self, velocity):
    	self.motors.sendW(velocity)
    
    def getLaserData(self):
        values = self.laser.getLaserData().values
        return values[270:360:1] + values[0:1] + values[1:90:1]
    
    def getImage(self):
    	return self.camera.getImage().data
    
    def getPose3d(self):
    	return self.odometry.getPose3d()
    
    def getBoundingBoxes(self, img):
    	rows = img.shape[0]
    	cols = img.shape[1]
    	detections = self.net.detect(img)
    	bounding_boxes = []
    	for detection in detections:
    		bounding_box = BoundingBox(
    			int(detection[1]),
    			LABEL_MAP[int(detection[1])],
    			float(detection[2]),
    			detection[3]*cols,
    			detection[4]*rows,
    			detection[5]*cols,
    			detection[6]*rows)
    		bounding_boxes.append(bounding_box)
    	return bounding_boxes


class ThreadHAL(threading.Thread):
    def __init__(self, executor):
        super(ThreadHAL, self).__init__()
        self.executor = executor

    def run(self):
    	try:
    	    self.executor.spin()
    	finally:
    	    self.executor.shutdown()
