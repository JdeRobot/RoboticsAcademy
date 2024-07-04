import cv2
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class HAL:
    def __init__(self):
        self.cameraCapture = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        self.image_subscriber = None
        self.start_ros_subscription()

    def start_ros_subscription(self):
        rclpy.init()
        node = rclpy.create_node('image_subscriber')
        self.image_subscriber = node.create_subscription(
            Image,
            'v4l2/raw_image',
            self.image_callback,
            10  
        )
        rclpy.spin(node)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('ROS Image', frame)
        cv2.waitKey(1)  

    def stop_ros_subscription(self):
        if self.image_subscriber:
            self.image_subscriber.destroy()
        rclpy.shutdown()

    def getImage(self):
        success, frame = self.cameraCapture.read()
        return frame

    def __del__(self):
        self.stop_ros_subscription()


