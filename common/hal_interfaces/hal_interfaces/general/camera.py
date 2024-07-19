from rclpy.node import Node
import sensor_msgs.msg
from math import pi as PI
import numpy as np
import cv_bridge
import cv2

### AUXILIARY FUNCTIONS ###

MAXRANGE = 8  # max length received from imageD
MINRANGE = 0


class Image:

    def __init__(self):

        self.height = 480  # Image height [pixels]
        self.width = 640  # Image width [pixels]
        self.timeStamp = 0  # Time stamp [s] */
        self.format = ""  # Image format string (RGB8, BGR,...)
        self.data = np.zeros(
            (self.height, self.width, 3), np.uint8
        )  # The image data itself
        self.data.shape = self.height, self.width, 3

    def __str__(self):
        s = (
            "Image: {\n   height: "
            + str(self.height)
            + "\n   width: "
            + str(self.width)
        )
        s = s + "\n   format: " + self.format + "\n   timeStamp: " + str(self.timeStamp)
        s = s + "\n   data: " + str(self.data) + "\n}"
        return s


def imageMsg2Image(img, bridge):

    if len(img.data) == 0:
        return None

    image = Image()

    image.width = img.width
    image.height = img.height
    image.format = "BGR8"
    image.timeStamp = img.header.stamp.sec + (img.header.stamp.nanosec * 1e-9)
    cv_image = 0
    if img.encoding[-2:] == "C1":
        gray_img_buff = bridge.imgmsg_to_cv2(img, img.encoding)
        cv_image = depthToRGB8(gray_img_buff, img.encoding)
    else:
        cv_image = bridge.imgmsg_to_cv2(img, "bgr8")

    image.data = cv_image
    return image


### HAL INTERFACE ###
class CameraNode(Node):

    def __init__(self, topic):
        super().__init__("camera_node")
        self.sub = self.create_subscription(
            sensor_msgs.msg.Image, topic, self.listener_callback, 10
        )
        self.last_img_ = sensor_msgs.msg.Image()
        self.bridge_ = cv_bridge.CvBridge()

    def listener_callback(self, msg):
        self.last_img_ = msg

    def getImage(self):
        return imageMsg2Image(self.last_img_, self.bridge_)
