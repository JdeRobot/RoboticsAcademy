import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import cv2

current_frame = None  # Global variable to store the frame

class WebcamSubscriber(Node):
    def __init__(self):
        super().__init__('webcam_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        global current_frame
        self.get_logger().info('Receiving video frame')
        current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def run_webcam_node():
    
    webcam_subscriber = WebcamSubscriber()

    rclpy.spin(webcam_subscriber)
    webcam_subscriber.destroy_node()
    

# Start the ROS2 node in a separate thread
thread = threading.Thread(target=run_webcam_node)
thread.start()

def getImage():
    global current_frame
    return current_frame
    
uploaded_vid_capture = cv2.VideoCapture('/workspace/code/demo_video/video.mp4')
frame_number = 0
def getVid():
        # After the frame has been inferred on, the frame number gets incremented in exercise.py
    uploaded_vid_capture.set(cv2.CAP_PROP_POS_FRAMES, frame_number)
    success, img = uploaded_vid_capture.read()
    return success, img

