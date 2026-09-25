from rclpy.node import Node
import sensor_msgs.msg
import cv_bridge


class DepthCameraNode(Node):
    """Depth of an RGBD camera as raw float32 meters, plus its camera intrinsics."""

    def __init__(self, image_topic, info_topic, node_name="hal_depth_camera_node"):
        super().__init__(node_name)
        self.bridge_ = cv_bridge.CvBridge()
        self.depth = None
        self.frame_id = None
        self.k = None
        self.create_subscription(
            sensor_msgs.msg.Image, image_topic, self.__depth_cb, 10
        )
        self.create_subscription(
            sensor_msgs.msg.CameraInfo, info_topic, self.__info_cb, 10
        )

    def __depth_cb(self, msg):
        self.depth = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.frame_id = msg.header.frame_id

    def __info_cb(self, msg):
        self.k = msg.k
