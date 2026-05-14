"""
HAL
Visual Odometry 3D
Envía:
- imagen input
- pose 3D actual
- estimated trajectory
- ground truth trajectory
"""

import rclpy
import sys
import threading
import time
import json
import traceback

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node

import WebGUI


# =========================================================
# IMAGE PUBLISHER
# =========================================================
class InputPublisher(Node):

    def __init__(self):
        super().__init__("input_publisher")
        self.image_pub = self.create_publisher(Image, "/input/image_raw", 10)
        self.bridge = CvBridge()

    def publish_image(self, cv_image):
        if cv_image is None:
            return

        try:
            ros_image = self.bridge.cv2_to_imgmsg(
                cv_image,
                encoding="bgr8"
            )
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera"
            self.image_pub.publish(ros_image)

        except CvBridgeError:
            self.get_logger().warning("CvBridge error")


# =========================================================
# ROS INIT
# =========================================================
if not rclpy.ok():
    rclpy.init(args=sys.argv)

input_publisher = InputPublisher()

executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(input_publisher)


def spin():
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.01)
        time.sleep(1 / 60.0)


threading.Thread(target=spin, daemon=True).start()


# =========================================================
# INTERNAL PAYLOAD
# =========================================================
def _send_payload(data):
    try:
        gui = getattr(WebGUI, "gui", None)

        if gui is not None and hasattr(gui, "payload"):
            for k, v in data.items():
                gui.payload[k] = json.dumps(v)
            return True

        if hasattr(WebGUI, "sendData"):
            WebGUI.sendData(data)
            return True

    except Exception as e:
        print("[HAL ERROR]", e)
        traceback.print_exc()

    return False


# =========================================================
# PUBLIC API
# =========================================================
def publish_input_image(img):
    input_publisher.publish_image(img)


def publish_pose3d(position, rotation=None):
    """
    position: [x,y,z]
    rotation: optional 3x3 or yaw
    """
    payload = {
        "camera_position": position
    }

    if rotation is not None:
        payload["camera_rotation"] = rotation

    _send_payload(payload)


def publish_estimated_path(path):
    """
    path = [[x,y,z], ...]
    """
    _send_payload({
        "estimated_path": path
    })


def publish_ground_truth_path(path):
    """
    path = [[x,y,z], ...]
    """
    _send_payload({
        "ground_truth_path": path
    })


def publish_all(position, estimated_path, gt_path):
    """
    helper principal
    """
    _send_payload({
        "camera_position": position,
        "estimated_path": estimated_path,
        "ground_truth_path": gt_path
    })