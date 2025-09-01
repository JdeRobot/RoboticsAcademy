import unittest
from unittest.mock import MagicMock, patch
import numpy as np
from hal_interfaces.general.camera import CameraNode, Image, imageMsg2Image
from tests.mocks.mock_ros_messages import MockROSImage

class TestCameraInterface(unittest.TestCase):
    @patch('rclpy.create_node')
    @patch('rclpy.init')
    def setUp(self, mock_init, mock_create_node):
        # Mock ROS initialization
        self.camera_node = CameraNode("test_topic")
        self.camera_node.bridge_ = MagicMock()
        
    def test_image_initialization(self):
        image = Image()
        self.assertEqual(image.height, 480)
        self.assertEqual(image.width, 640)
        self.assertEqual(image.format, "")
        self.assertEqual(image.timeStamp, 0)
        self.assertEqual(image.data.shape, (480, 640, 3))
        
    def test_get_image_empty_message(self):
        # Test with empty message
        self.camera_node.last_img_ = MockROSImage()
        self.camera_node.last_img_.data = bytes()  # Empty data
        
        # Empty message should return None
        result = self.camera_node.getImage()
        self.assertIsNone(result)
        
    def test_get_image_with_data(self):
        # Create a mock image message with data
        img_msg = MockROSImage(height=480, width=640)
        img_msg.header.stamp.sec = 10
        img_msg.header.stamp.nanosec = 500000000
        
        self.camera_node.last_img_ = img_msg
        self.camera_node.bridge_.imgmsg_to_cv2 = MagicMock(return_value=np.ones((480, 640, 3)))
        
        result = self.camera_node.getImage()
        self.assertIsNotNone(result)
        self.assertEqual(result.height, 480)
        self.assertEqual(result.width, 640)
        self.assertEqual(result.format, "BGR8")
        self.assertEqual(result.timeStamp, 10.5)  # 10 seconds + 500ms
