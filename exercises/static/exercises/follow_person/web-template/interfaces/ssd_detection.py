import cv2

FROZEN_GRAPH = "/RoboticsAcademy/exercises/static/exercises/follow_person/web-template/interfaces/pretrained_model/ssd_inception_v2_coco.pb"
PB_TXT = "/RoboticsAcademy/exercises/static/exercises/follow_person/web-template/interfaces/pretrained_model/ssd_inception_v2_coco.pbtxt"
SIZE = 300

class BoundingBox:
	def __init__(self, identifier, class_id, score, xmin, ymin, xmax, ymax):
		self.id = identifier
		self.class_id = class_id
		self.score = score
		self.xmin = xmin
		self.ymin = ymin
		self.xmax = xmax
		self.ymax = ymax
	
	def __str__(self):
		s = "[id:{}\nclass:{}\nscore:{}\nxmin:{}\nymin:{}\nxmax:{}\nymax:{}\n".format(
			self.id, self.class_id, self.score, self.xmin, self.ymin, self.xmax, self.ymax)
		return s
		
class NeuralNetwork:

	def __init__(self):
		self.net = cv2.dnn.readNetFromTensorflow(FROZEN_GRAPH, PB_TXT)
		
	def detect(self, img):
		rows = img.shape[0]
		cols = img.shape[1]
		self.net.setInput(cv2.dnn.blobFromImage(img, 1.0/127.5, (SIZE, SIZE), (127.5, 127.5, 127.5), swapRB=True, crop=False))
		cvOut = self.net.forward()
		
		return cvOut[0, 0, :, :]
