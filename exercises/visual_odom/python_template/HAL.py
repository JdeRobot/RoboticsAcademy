"""
HAL (Hardware Abstraction Layer) for Basic Computer Vision Exercise
Now extended with Visual Odometry support (didactic trajectory + drift)
"""

import rclpy
import sys
import threading
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node

# NEW: WebGUI bridge
import WebGUI


# =========================================================
# INPUT IMAGE PUBLISHER (UNCHANGED)
# =========================================================

class InputPublisher(Node):

    def __init__(self):
        super().__init__("input_publisher")
        self.publisher = self.create_publisher(Image, "/input/image_raw", 10)
        self.bridge = CvBridge()
        self.get_logger().info("Input publisher initialized on /input/image_raw")

    def publish_image(self, cv_image):

        if cv_image is None:
            self.get_logger().warn("None image skipped")
            return

        try:
            if len(cv_image.shape) != 3 or cv_image.shape[2] != 3:
                self.get_logger().error(f"Bad shape: {cv_image.shape}")
                return

            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.publisher.publish(ros_image)

        except CvBridgeError as e:
            self.get_logger().error(f"Bridge error: {e}")

        except Exception as e:
            self.get_logger().error(f"Publish error: {e}")


# =========================================================
# VISUAL ODOMETRY STATE (NEW)
# =========================================================

class VisualOdometry:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0

        self.prev_x = 0.0
        self.prev_y = 0.0

        self.drift = 0.0

        self.lock = threading.Lock()

    def update(self, dx, dy):

        with self.lock:
            # integrate motion
            self.x += dx
            self.y += dy

            # drift = distance from start
            self.drift = (self.x ** 2 + self.y ** 2) ** 0.5

            return {
                "x": self.x,
                "y": self.y,
                "drift": self.drift
            }


# =========================================================
# GLOBAL STATE
# =========================================================

vo = VisualOdometry()

input_publisher = None
executor = None


# =========================================================
# ROS2 INIT
# =========================================================

if not rclpy.ok():
    rclpy.init(args=sys.argv)

input_publisher = InputPublisher()

executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(input_publisher)


def __auto_spin():
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0)
        except Exception as e:
            print(f"Spin error: {e}", file=sys.stderr)

        time.sleep(1 / 30.0)


threading.Thread(
    target=__auto_spin,
    daemon=True,
    name="hal_spin_thread"
).start()


# =========================================================
# IMAGE API
# =========================================================

def publish_input_image(cv_image):
    if input_publisher:
        input_publisher.publish_image(cv_image)


# =========================================================
# NEW: VISUAL ODOMETRY API
# =========================================================

def publish_user_motion(dx, dy):
    """
    Called from vision script (optical flow result)
    """

    odom = vo.update(dx, dy)

    # Send to WebGUI (RIGHT PANEL)
    WebGUI.sendOdom(odom)