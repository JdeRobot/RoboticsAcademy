import cv2

FROZEN_GRAPH = "/RoboticsAcademy/exercises/static/exercises/follow_person_newmanager/python_template/ros2_humble/hal_interfaces/pretrained_model/ssd_inception_v2_coco.pb"
PB_TXT = "/RoboticsAcademy/exercises/static/exercises/follow_person_newmanager/python_template/ros2_humble/hal_interfaces/pretrained_model/ssd_inception_v2_coco.pbtxt"
SIZE = 300

LABEL_MAP = {
    0: "unlabeled",
    1: "person",
    2: "bicycle",
    3: "car",
    4: "motorcycle",
    5: "airplane",
    6: "bus",
    7: "train",
    8: "truck",
    9: "boat",
    10: "traffic",
    11: "fire",
    12: "street",
    13: "stop",
    14: "parking",
}

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
	
    # Get bounding boxes function
    def getBoundingBoxes(self, img):
        rows = img.shape[0]
        cols = img.shape[1]
        detections = self.detect(img)
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
