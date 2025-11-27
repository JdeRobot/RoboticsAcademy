"""
HAL (Hardware Abstraction Layer) for Basic Computer Vision Exercise
Bridges frontend webcam images to ROS2 /webcam/image_raw topic
"""

import rclpy
import sys
import threading
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node


class WebcamPublisher(Node):
    """Publishes webcam images received from frontend to ROS2 topic"""

    def __init__(self):
        super().__init__("webcam_publisher")
        self.publisher = self.create_publisher(Image, "/webcam/image_raw", 10)
        self.bridge = CvBridge()
        self.get_logger().info("Webcam publisher initialized on /webcam/image_raw")

    def publish_image(self, cv_image):
        """
        Publish a CV image to the ROS2 topic

        Args:
            cv_image: OpenCV image (numpy array) in BGR format
        """
        if cv_image is None:
            self.get_logger().warn("Attempted to publish None image, skipping")
            return

        try:
            # Validate image has correct shape
            if len(cv_image.shape) != 3 or cv_image.shape[2] != 3:
                self.get_logger().error(
                    f"Invalid image shape: {cv_image.shape}. Expected (H, W, 3)"
                )
                return

            # Convert OpenCV image to ROS2 Image message
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.publisher.publish(ros_image)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge conversion error: {e}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")


# Publishing frequency for executor spin
SPIN_FREQUENCY = 30.0  # Hz


def custom_thread_excepthook(args):
    """Mutes exceptions from ROS2 spin threads to avoid console spam"""
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook


def __auto_spin() -> None:
    """Auto-spin ROS2 executor in background thread"""
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0)
        except Exception as e:
            # Log unexpected errors during spin
            print(f"Error in ROS2 executor spin: {e}", file=sys.stderr)
        time.sleep(1 / SPIN_FREQUENCY)


# Initialize ROS2 context
if not rclpy.ok():
    rclpy.init(args=sys.argv)

# Create webcam publisher node
webcam_publisher = WebcamPublisher()

# Setup executor and start background spin thread
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(webcam_publisher)
executor_thread = threading.Thread(
    target=__auto_spin, daemon=True, name="hal_spin_thread"
)
executor_thread.start()


def publish_webcam_image(cv_image):
    """
    Publish webcam image to ROS2 topic

    This function is called by WebGUI when receiving webcam frames from the browser.
    The image is published to /webcam/image_raw for ROS2 users to subscribe to.

    Args:
        cv_image: OpenCV image (numpy array) in BGR format
    """
    if webcam_publisher is not None:
        webcam_publisher.publish_image(cv_image)
    else:
        print("Warning: webcam_publisher not initialized", file=sys.stderr)
