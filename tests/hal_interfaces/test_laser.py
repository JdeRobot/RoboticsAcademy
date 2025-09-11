import unittest
from math import pi as PI
import numpy as np
from unittest.mock import MagicMock

from hal_interfaces.general.laser import LaserData, laserScan2LaserData, LaserNode


class MockLaserScan:
    """Creates a mock LaserScan message with realistic data"""

    def __init__(
        self,
        ranges=None,
        angle_min=-PI / 2,
        angle_max=PI / 2,
        range_min=0.1,
        range_max=10.0,
        sec=10,
        nanosec=500000000,
    ):
        """
        Initialize a mock LaserScan with realistic data

        Args:
            ranges: List of range values (defaults to a 180-degree scan with 181 points)
            angle_min: Start angle in radians (default -PI/2)
            angle_max: End angle in radians (default PI/2)
            range_min: Minimum range in meters (default 0.1)
            range_max: Maximum range in meters (default 10.0)
            sec: Seconds part of timestamp (default 10)
            nanosec: Nanoseconds part of timestamp (default 500000000 - 0.5 seconds)
        """
        # Create a basic header
        self.header = MagicMock()
        self.header.stamp.sec = sec
        self.header.stamp.nanosec = nanosec

        # Set scan parameters
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.range_min = range_min
        self.range_max = range_max

        # Create ranges - if None, create a simple pattern
        if ranges is None:
            # Create 181 points for 180 degrees (-90° to +90°)
            # Pattern: closer in front (5m), farther on sides (8m)
            num_points = 181
            middle_index = num_points // 2

            # Initialize with 8m distance
            ranges = [8.0] * num_points

            # Make middle points (front of robot) closer (5m)
            # and gradually transition to sides
            for i in range(num_points):
                # Distance from center point
                dist_from_center = abs(i - middle_index) / middle_index
                # Closer at center, farther at edges
                ranges[i] = 5.0 + 3.0 * dist_from_center

            # Add some "obstacles" at specific angles
            ranges[30] = 2.0  # Obstacle at -60 degrees
            ranges[90] = 4.0  # Obstacle at 0 degrees (directly in front)
            ranges[150] = 3.0  # Obstacle at +60 degrees

        self.ranges = ranges


class TestLaserData(unittest.TestCase):
    """Test the LaserData class"""

    def test_initialization(self):
        """Test that LaserData initializes with default values"""
        laser_data = LaserData()

        # Check default values
        self.assertEqual(laser_data.values, [])
        self.assertEqual(laser_data.minAngle, 0)
        self.assertEqual(laser_data.maxAngle, 0)
        self.assertEqual(laser_data.minRange, 0)
        self.assertEqual(laser_data.maxRange, 0)
        self.assertEqual(laser_data.timeStamp, 0)

    def test_string_representation(self):
        """Test the string representation of LaserData"""
        laser_data = LaserData()
        laser_data.values = [1.0, 2.0, 3.0]
        laser_data.minAngle = -PI / 2
        laser_data.maxAngle = PI / 2
        laser_data.minRange = 0.1
        laser_data.maxRange = 10.0
        laser_data.timeStamp = 10.5

        # Convert to string and check content
        str_repr = str(laser_data)
        self.assertIn("minAngle: -1.57", str_repr)
        self.assertIn("maxAngle: 1.57", str_repr)
        self.assertIn("minRange: 0.1", str_repr)
        self.assertIn("maxRange: 10.0", str_repr)
        self.assertIn("timeStamp: 10.5", str_repr)
        self.assertIn("values: [1.0, 2.0, 3.0]", str_repr)


class TestLaserScan2LaserData(unittest.TestCase):
    """Test the laserScan2LaserData conversion function"""

    def test_basic_conversion(self):
        """Test basic conversion from LaserScan to LaserData"""
        # Create a mock scan with default values
        mock_scan = MockLaserScan()

        # Convert to LaserData
        laser_data = laserScan2LaserData(mock_scan)

        # Check that values were copied correctly
        self.assertEqual(laser_data.values, mock_scan.ranges)

        # Check angle conversion (ROS to JdeRobot convention)
        self.assertAlmostEqual(laser_data.minAngle, mock_scan.angle_min + PI / 2)
        self.assertAlmostEqual(laser_data.maxAngle, mock_scan.angle_max + PI / 2)

        # Check range limits
        self.assertEqual(laser_data.minRange, mock_scan.range_min)
        self.assertEqual(laser_data.maxRange, mock_scan.range_max)

        # Check timestamp conversion
        expected_timestamp = mock_scan.header.stamp.sec + (
            mock_scan.header.stamp.nanosec * 1e-9
        )
        self.assertAlmostEqual(laser_data.timeStamp, expected_timestamp)

    def test_different_angle_ranges(self):
        """Test conversion with different angle ranges"""
        # Test with 360 degree scan (-PI to PI)
        mock_scan = MockLaserScan(angle_min=-PI, angle_max=PI)
        laser_data = laserScan2LaserData(mock_scan)

        # Check angle conversion
        self.assertAlmostEqual(laser_data.minAngle, -PI + PI / 2)  # -PI/2
        self.assertAlmostEqual(laser_data.maxAngle, PI + PI / 2)  # 3PI/2

    def test_custom_range_values(self):
        """Test with custom range values"""
        # Create custom ranges representing different scenarios
        custom_ranges = [float("inf"), 5.0, 3.0, 2.0, 1.0, 2.0, 3.0, 5.0, float("inf")]
        mock_scan = MockLaserScan(ranges=custom_ranges)

        laser_data = laserScan2LaserData(mock_scan)

        # Verify ranges were copied correctly
        self.assertEqual(len(laser_data.values), len(custom_ranges))
        for i in range(len(custom_ranges)):
            self.assertEqual(laser_data.values[i], custom_ranges[i])


class TestLaserNode(unittest.TestCase):
    """Test the LaserNode class"""

    def test_callback_and_get_data(self):
        """Test that listener_callback updates data and getLaserData returns it"""
        # Create a LaserNode but mock the Node initialization and subscription
        # to avoid ROS dependency
        LaserNode.__init__ = MagicMock(return_value=None)
        node = LaserNode("mock_topic")

        # Create a mock scan
        mock_scan = MockLaserScan()

        # Mock the last_scan_ attribute
        node.last_scan_ = mock_scan

        # Call getLaserData and verify results
        laser_data = node.getLaserData()

        # Verify results match what we expect from conversion
        self.assertEqual(laser_data.values, mock_scan.ranges)
        self.assertAlmostEqual(laser_data.minAngle, mock_scan.angle_min + PI / 2)
        self.assertAlmostEqual(laser_data.maxAngle, mock_scan.angle_max + PI / 2)

    def test_listener_callback(self):
        """Test that listener_callback updates internal state"""
        # Create a LaserNode but mock the Node initialization
        LaserNode.__init__ = MagicMock(return_value=None)
        node = LaserNode("mock_topic")

        # Set initial empty scan
        node.last_scan_ = None

        # Create a mock scan and call callback
        mock_scan = MockLaserScan()
        node.listener_callback(mock_scan)

        # Verify last_scan_ was updated
        self.assertEqual(node.last_scan_, mock_scan)


if __name__ == "__main__":
    unittest.main()
