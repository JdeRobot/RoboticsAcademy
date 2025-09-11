import unittest
import numpy as np
from unittest.mock import MagicMock, patch

from hal_interfaces.general.camera import Image, imageMsg2Image, CameraNode

class MockROSImage:
    def __init__(self, width=640, height=480, encoding="bgr8", data=None, sec=0, nanosec=0):
        self.width = width
        self.height = height
        self.encoding = encoding
        self.header = type('', (), {})()
        self.header.stamp = type('', (), {})()
        self.header.stamp.sec = sec
        self.header.stamp.nanosec = nanosec
        self.data = data if data is not None else bytes([1] * (width * height * 3))

class TestImageClass(unittest.TestCase):
    def test_image_initialization(self):
        img = Image()
        self.assertEqual(img.height, 480)
        self.assertEqual(img.width, 640)
        self.assertEqual(img.data.shape, (480, 640, 3))
        self.assertEqual(img.format, "")
        self.assertEqual(img.timeStamp, 0)
        self.assertIsInstance(str(img), str)

class TestImageMsg2Image(unittest.TestCase):
    @patch("hal_interfaces.general.camera.cv_bridge.CvBridge")
    def test_image_conversion_bgr8(self, MockBridge):
        mock_img = MockROSImage()
        mock_bridge = MockBridge.return_value
        mock_bridge.imgmsg_to_cv2.return_value = np.ones((480, 640, 3), np.uint8)
        result = imageMsg2Image(mock_img, mock_bridge)
        self.assertIsInstance(result, Image)
        self.assertEqual(result.width, 640)
        self.assertEqual(result.height, 480)
        self.assertEqual(result.format, "BGR8")
        self.assertTrue(np.array_equal(result.data, np.ones((480, 640, 3), np.uint8)))

    @patch("hal_interfaces.general.camera.cv_bridge.CvBridge")
    def test_image_conversion_empty_data(self, MockBridge):
        mock_img = MockROSImage(data=bytes())
        mock_bridge = MockBridge.return_value
        result = imageMsg2Image(mock_img, mock_bridge)
        self.assertIsNone(result)

class TestCameraNode(unittest.TestCase):
    @patch("hal_interfaces.general.camera.cv_bridge.CvBridge")
    #@patch("hal_interfaces.general.camera.sensor_msgs.msg.Image")
    def test_listener_and_get_image(self, MockBridge):
        node = CameraNode("test_topic")
        mock_img = MockROSImage()
        node.listener_callback(mock_img)
        mock_bridge = MockBridge.return_value
        mock_bridge.imgmsg_to_cv2.return_value = np.ones((480, 640, 3), np.uint8)
        result = node.getImage()
        self.assertIsInstance(result, Image)
        self.assertEqual(result.width, 640)
        self.assertEqual(result.height, 480)

if __name__ == "__main__":
    unittest.main()