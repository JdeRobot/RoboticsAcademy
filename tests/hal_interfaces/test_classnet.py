import unittest
import numpy as np
from unittest.mock import MagicMock, patch
import os

from hal_interfaces.general.classnet import BoundingBox, NeuralNetwork, LABEL_MAP


class TestBoundingBox(unittest.TestCase):
    """Test the BoundingBox class functionality"""

    def setUp(self):
        """Set up a sample bounding box for tests"""
        self.bbox = BoundingBox(
            identifier=1,  # ID
            class_id=1,  # Person class
            score=0.95,  # 95% confidence
            xmin=100.0,  # Left boundary
            ymin=150.0,  # Top boundary
            xmax=300.0,  # Right boundary
            ymax=400.0,  # Bottom boundary
        )

    def test_initialization(self):
        """Test that BoundingBox initializes with correct values"""
        self.assertEqual(self.bbox.id, 1)
        self.assertEqual(self.bbox.class_id, 1)
        self.assertEqual(self.bbox.score, 0.95)
        self.assertEqual(self.bbox.xmin, 100.0)
        self.assertEqual(self.bbox.ymin, 150.0)
        self.assertEqual(self.bbox.xmax, 300.0)
        self.assertEqual(self.bbox.ymax, 400.0)

    def test_string_representation(self):
        """Test the string representation of BoundingBox"""
        bbox_str = str(self.bbox)
        self.assertIn("id:1", bbox_str)
        self.assertIn("class:1", bbox_str)
        self.assertIn("score:0.95", bbox_str)
        self.assertIn("xmin:100.0", bbox_str)
        self.assertIn("ymin:150.0", bbox_str)
        self.assertIn("xmax:300.0", bbox_str)
        self.assertIn("ymax:400.0", bbox_str)


class TestNeuralNetworkMocked(unittest.TestCase):
    """Test NeuralNetwork class with mocked OpenCV DNN"""

    @patch("hal_interfaces.general.classnet.cv2.dnn.readNetFromTensorflow")
    def setUp(self, mock_read_net):
        """Set up a NeuralNetwork with mocked OpenCV DNN"""
        # Configure the mock net
        self.mock_net = MagicMock()
        mock_read_net.return_value = self.mock_net

        # Create test image (RGB)
        self.test_img = np.ones((480, 640, 3), dtype=np.uint8) * 128

        # Mock detection results (3 objects)
        # Format: [image_id, label, confidence, xmin, ymin, xmax, ymax]
        self.mock_detections = np.array(
            [
                [0, 1, 0.95, 0.1, 0.2, 0.3, 0.4],  # Person with 95% confidence
                [0, 2, 0.85, 0.4, 0.5, 0.6, 0.7],  # Bicycle with 85% confidence
                [0, 3, 0.75, 0.7, 0.8, 0.9, 1.0],  # Car with 75% confidence
            ]
        )

        # Initialize with mocked components
        self.neural_network = NeuralNetwork()

    def test_detect_method(self):
        """Test the detect method with mocked responses"""
        # Configure mock to return our test detections
        self.mock_net.forward.return_value = np.array([[self.mock_detections]])

        # Call detect method
        result = self.neural_network.detect(self.test_img)

        # Verify blob creation and input
        self.mock_net.setInput.assert_called_once()
        self.mock_net.forward.assert_called_once()

        # Verify result is our mock data
        np.testing.assert_array_equal(result, self.mock_detections)

    def test_get_bounding_boxes(self):
        """Test the getBoundingBoxes method with mocked detections"""
        # Configure detect to return our mock data
        self.neural_network.detect = MagicMock(return_value=self.mock_detections)

        # Call getBoundingBoxes
        boxes = self.neural_network.getBoundingBoxes(self.test_img)

        # Verify detect was called
        self.neural_network.detect.assert_called_once_with(self.test_img)

        # Verify we got 3 boxes
        self.assertEqual(len(boxes), 3)

        # Check first box (person)
        self.assertEqual(boxes[0].id, 1)
        self.assertEqual(boxes[0].class_id, "person")  # Should map ID to label
        self.assertEqual(boxes[0].score, 0.95)
        self.assertEqual(boxes[0].xmin, 0.1 * 640)  # Scaled by image width
        self.assertEqual(boxes[0].ymin, 0.2 * 480)  # Scaled by image height
        self.assertEqual(boxes[0].xmax, 0.3 * 640)  # Scaled by image width
        self.assertEqual(boxes[0].ymax, 0.4 * 480)  # Scaled by image height

        # Check second box (bicycle)
        self.assertEqual(boxes[1].id, 2)
        self.assertEqual(boxes[1].class_id, "bicycle")

        # Check third box (car)
        self.assertEqual(boxes[2].id, 3)
        self.assertEqual(boxes[2].class_id, "car")


class TestLabelMap(unittest.TestCase):
    """Test the LABEL_MAP constant"""

    def test_label_map_entries(self):
        """Test that LABEL_MAP has the expected entries"""
        self.assertEqual(LABEL_MAP[1], "person")
        self.assertEqual(LABEL_MAP[2], "bicycle")
        self.assertEqual(LABEL_MAP[3], "car")
        self.assertEqual(LABEL_MAP[15], "bench")
        self.assertEqual(LABEL_MAP[62], "chair")


if __name__ == "__main__":
    unittest.main()
