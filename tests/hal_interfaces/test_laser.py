import unittest
from unittest.mock import MagicMock, patch
import numpy as np
from hal_interfaces.general.laser import LaserNode
from tests.mocks.mock_ros_messages import MockROSLaserScan

class TestLaserInterface(unittest.TestCase):
    @patch('rclpy.create_node')
    @patch('rclpy.init')
    def setUp(self, mock_init, mock_create_node):
        # Mock ROS initialization
        self.laser_node = LaserNode("test_topic")
        
    def test_laser_data_processing(self):
        # Create mock laser scan message
        scan_msg = MockROSLaserScan(
            angle_min=-1.57,  # -90 degrees
            angle_max=1.57,   # 90 degrees
            angle_increment=0.01,
            range_min=0.1,
            range_max=10.0,
            ranges=[5.0] * 315  # 315 points in the scan
        )
        
        # Set the mock message
        self.laser_node.last_scan_ = scan_msg
        
        # Get laser data
        laser_data = self.laser_node.getLaserData()
        
        # Verify laser data
        self.assertEqual(len(laser_data.values), 315)
        self.assertEqual(laser_data.minAngle, -1.57)
        self.assertEqual(laser_data.maxAngle, 1.57)
        self.assertEqual(laser_data.minRange, 0.1)
        self.assertEqual(laser_data.maxRange, 10.0)
