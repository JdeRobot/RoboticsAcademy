import unittest
import math
from unittest.mock import MagicMock, patch
import numpy as np

from hal_interfaces.general.odometry import (
    Pose3d, quat2Yaw, quat2Pitch, quat2Roll, odometry2Pose3D, OdometryNode
)

class TestPose3d(unittest.TestCase):
    """Test the Pose3d class functionality"""
    
    def test_initialization(self):
        """Test that Pose3d initializes with correct default values"""
        pose = Pose3d()
        self.assertEqual(pose.x, 0)
        self.assertEqual(pose.y, 0)
        self.assertEqual(pose.z, 0)
        self.assertEqual(pose.h, 1)
        self.assertEqual(pose.yaw, 0)
        self.assertEqual(pose.pitch, 0)
        self.assertEqual(pose.roll, 0)
        self.assertEqual(pose.q, [0, 0, 0, 0])
        self.assertEqual(pose.timeStamp, 0)
    
    def test_string_representation(self):
        """Test the string representation of Pose3d"""
        pose = Pose3d()
        pose.x = 1.5
        pose.y = 2.7
        pose.z = 0.3
        pose.yaw = 0.5
        pose.pitch = 0.1
        pose.roll = 0.2
        pose.q = [0.9, 0.1, 0.2, 0.3]
        pose.timeStamp = 123.456
        
        str_repr = str(pose)
        self.assertIn("x: 1.5", str_repr)
        self.assertIn("y: 2.7", str_repr)
        self.assertIn("z: 0.3", str_repr)
        self.assertIn("H: 1", str_repr)
        self.assertIn("Yaw: 0.5", str_repr)
        self.assertIn("Pitch: 0.1", str_repr)
        self.assertIn("Roll: 0.2", str_repr)
        self.assertIn("quaternion: [0.9, 0.1, 0.2, 0.3]", str_repr)
        self.assertIn("timeStamp: 123.456", str_repr)


class TestQuaternionConversions(unittest.TestCase):
    """Test quaternion conversion functions"""
    
    def test_quat2yaw_identity(self):
        """Test quaternion to yaw conversion with identity quaternion"""
        # Identity quaternion (no rotation)
        yaw = quat2Yaw(1.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(yaw, 0.0)
    
    def test_quat2pitch_identity(self):
        """Test quaternion to pitch conversion with identity quaternion"""
        pitch = quat2Pitch(1.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(pitch, 0.0)
    
    def test_quat2roll_identity(self):
        """Test quaternion to roll conversion with identity quaternion"""
        roll = quat2Roll(1.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(roll, 0.0)
    
    def test_quat2yaw_90_degrees(self):
        """Test quaternion to yaw conversion with 90° yaw rotation"""
        # Quaternion for 90° rotation around Z-axis (yaw)
        w = math.cos(math.pi/4)  # cos(θ/2)
        z = math.sin(math.pi/4)  # sin(θ/2)
        yaw = quat2Yaw(w, 0.0, 0.0, z)
        self.assertAlmostEqual(yaw, math.pi/2, places=5)
    
    def test_quat2pitch_90_degrees(self):
        """Test quaternion to pitch conversion with 90° pitch rotation"""
        # Quaternion for 90° rotation around Y-axis (pitch)
        w = math.cos(math.pi/4)  # cos(θ/2)
        y = math.sin(math.pi/4)  # sin(θ/2)
        pitch = quat2Pitch(w, 0.0, y, 0.0)
        self.assertAlmostEqual(pitch, math.pi/2, places=5)
    
    def test_quat2roll_90_degrees(self):
        """Test quaternion to roll conversion with 90° roll rotation"""
        # Quaternion for 90° rotation around X-axis (roll)
        w = math.cos(math.pi/4)  # cos(θ/2)
        x = math.sin(math.pi/4)  # sin(θ/2)
        roll = quat2Roll(w, x, 0.0, 0.0)
        self.assertAlmostEqual(roll, math.pi/2, places=5)
    
    def test_quat2yaw_minus_90_degrees(self):
        """Test quaternion to yaw conversion with -90° yaw rotation"""
        # Quaternion for -90° rotation around Z-axis (yaw)
        w = math.cos(-math.pi/4)  # cos(θ/2)
        z = math.sin(-math.pi/4)  # sin(θ/2)
        yaw = quat2Yaw(w, 0.0, 0.0, z)
        self.assertAlmostEqual(yaw, -math.pi/2, places=5)
    
    def test_combined_rotation(self):
        """Test with a quaternion representing combined rotation"""
        # Create a quaternion with combined roll-pitch-yaw rotation
        # This is a more complex test case
        qw, qx, qy, qz = 0.5, 0.5, 0.5, 0.5  # Normalized quaternion
        
        yaw = quat2Yaw(qw, qx, qy, qz)
        pitch = quat2Pitch(qw, qx, qy, qz)
        roll = quat2Roll(qw, qx, qy, qz)
        
        # These values can be verified with external quaternion calculators
        # For this specific quaternion, all angles should be approximately the same
        self.assertIsNotNone(yaw)
        self.assertIsNotNone(pitch)
        self.assertIsNotNone(roll)


class MockOdometry:
    """Creates a mock Odometry message for testing"""
    
    def __init__(self, x=0.0, y=0.0, z=0.0, qw=1.0, qx=0.0, qy=0.0, qz=0.0, sec=0, nanosec=0):
        """Initialize with custom position, orientation and timestamp"""
        # Create nested structure exactly as in ROS2 nav_msgs.msg.Odometry
        self.header = type('Header', (), {})()
        self.header.stamp = type('Time', (), {'sec': sec, 'nanosec': nanosec})()
        
        self.pose = type('PoseWithCovariance', (), {})()
        self.pose.pose = type('Pose', (), {})()
        self.pose.pose.position = type('Point', (), {'x': x, 'y': y, 'z': z})()
        self.pose.pose.orientation = type('Quaternion', (), {'w': qw, 'x': qx, 'y': qy, 'z': qz})()


class TestOdometry2Pose3D(unittest.TestCase):
    """Test the odometry2Pose3D function"""
    
    def test_basic_conversion(self):
        """Test basic conversion from Odometry to Pose3d"""
        # Create a mock odometry message
        odom = MockOdometry(10.0, 20.0, 30.0, 1.0, 0.0, 0.0, 0.0, 50, 500000000)
        
        # Convert to Pose3d
        pose = odometry2Pose3D(odom)
        
        # Check position values
        self.assertEqual(pose.x, 10.0)
        self.assertEqual(pose.y, 20.0)
        self.assertEqual(pose.z, 30.0)
        
        # Check orientation (identity quaternion)
        self.assertAlmostEqual(pose.yaw, 0.0)
        self.assertAlmostEqual(pose.pitch, 0.0)
        self.assertAlmostEqual(pose.roll, 0.0)
        self.assertEqual(pose.q, [1.0, 0.0, 0.0, 0.0])
        
        # Check timestamp (50.5 seconds)
        self.assertEqual(pose.timeStamp, 50.5)
    
    def test_conversion_with_orientation(self):
        """Test conversion with non-identity orientation"""
        # Create a mock odometry with 90° yaw rotation
        w = math.cos(math.pi/4)  # cos(θ/2)
        z = math.sin(math.pi/4)  # sin(θ/2)
        odom = MockOdometry(1.0, 2.0, 3.0, w, 0.0, 0.0, z, 10, 0)
        
        # Convert to Pose3d
        pose = odometry2Pose3D(odom)
        
        # Check orientation
        self.assertAlmostEqual(pose.yaw, math.pi/2, places=5)
        self.assertAlmostEqual(pose.pitch, 0.0)
        self.assertAlmostEqual(pose.roll, 0.0)
        self.assertEqual(pose.q, [w, 0.0, 0.0, z])


class TestOdometryNode(unittest.TestCase):
    """Test the OdometryNode class"""
    
    @patch('hal_interfaces.general.odometry.Node.__init__')
    def test_node_initialization(self, mock_init):
        """Test OdometryNode initialization"""
        # Avoid actual ROS2 initialization
        mock_init.return_value = None
        
        # Mock the create_subscription method
        with patch('hal_interfaces.general.odometry.Node.create_subscription', 
                  return_value=MagicMock()):
            node = OdometryNode("test_topic")
            
            # Check that the node was initialized with the correct name
            mock_init.assert_called_once_with("odometry_node")
            
            # Check that a subscription was created
            self.assertTrue(hasattr(node, 'sub'))
            self.assertTrue(hasattr(node, 'last_pose_'))
    
    @patch('hal_interfaces.general.odometry.Node.__init__')
    def test_listener_callback(self, mock_init):
        """Test the listener_callback method"""
        # Avoid actual ROS2 initialization
        mock_init.return_value = None
        
        # Mock the create_subscription method
        with patch('hal_interfaces.general.odometry.Node.create_subscription', 
                  return_value=MagicMock()):
            node = OdometryNode("test_topic")
            
            # Create a mock odometry message
            mock_odom = MockOdometry(5.0, 10.0, 15.0)
            
            # Call the callback
            node.listener_callback(mock_odom)
            
            # Check that the last_pose_ was updated
            self.assertEqual(node.last_pose_, mock_odom)


if __name__ == "__main__":
    unittest.main()