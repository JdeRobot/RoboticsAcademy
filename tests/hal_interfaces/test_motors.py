import unittest
from unittest.mock import MagicMock, patch
from hal_interfaces.general.motors import MotorsNode

class TestMotorsInterface(unittest.TestCase):
    @patch('rclpy.create_node')
    @patch('rclpy.init')
    def setUp(self, mock_init, mock_create_node):
        # Mock ROS initialization
        self.motors_node = MotorsNode("test_topic")
        self.motors_node.publisher_ = MagicMock()
        
    def test_send_cmd_vel(self):
        # Test sending velocity commands
        self.motors_node.sendVelocities(1.0, 0.5)
        
        # Check if publisher was called with correct values
        self.motors_node.publisher_.publish.assert_called_once()
        args = self.motors_node.publisher_.publish.call_args[0][0]
        self.assertAlmostEqual(args.linear.x, 1.0)
        self.assertAlmostEqual(args.angular.z, 0.5)
        
    def test_send_zero_velocity(self):
        # Test sending zero velocity (stop command)
        self.motors_node.sendVelocities(0.0, 0.0)
        
        # Check if publisher was called with correct values
        self.motors_node.publisher_.publish.assert_called_once()
        args = self.motors_node.publisher_.publish.call_args[0][0]
        self.assertAlmostEqual(args.linear.x, 0.0)
        self.assertAlmostEqual(args.angular.z, 0.0)
