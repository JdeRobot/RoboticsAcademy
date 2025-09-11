import unittest
from unittest.mock import MagicMock, patch, call
import rclpy
from geometry_msgs.msg import Twist

from hal_interfaces.general.motors import MotorsNode


class TestMotorsNode(unittest.TestCase):
    """Test the MotorsNode class for motor control"""

    def setUp(self):
        """Set up test environment"""
        # Patch rclpy.init and rclpy.shutdown to avoid ROS initialization
        with patch("hal_interfaces.general.motors.Node.__init__") as mock_init:
            mock_init.return_value = None

            # Mock create_publisher to avoid ROS calls
            with patch(
                "hal_interfaces.general.motors.Node.create_publisher"
            ) as mock_create_pub:
                # Return a mock publisher
                mock_publisher = MagicMock()
                mock_create_pub.return_value = mock_publisher

                # Now create the MotorsNode
                self.node = MotorsNode("/cmd_vel", 1.0, 2.0)

                # Manually set the publisher since we mocked it
                self.node.pub = mock_publisher

        # Initialize last_twist as a new Twist
        self.node.last_twist = Twist()

    def test_initialization(self):
        """Test that MotorsNode initializes properly"""
        with patch("hal_interfaces.general.motors.Node.__init__") as mock_init:
            mock_init.return_value = None

            with patch(
                "hal_interfaces.general.motors.Node.create_publisher"
            ) as mock_create_pub:
                # Return a mock publisher
                mock_publisher = MagicMock()
                mock_create_pub.return_value = mock_publisher

                # Create node with specific topic
                node = MotorsNode("/custom_cmd_vel", 0.5, 1.0)

                # Check that Node.__init__ was called with correct name
                mock_init.assert_called_once_with("motors_node")

                # Check that a Twist message was initialized
                self.assertIsInstance(node.last_twist, Twist)

    def test_send_v(self):
        """Test sending linear velocity"""
        # Test with zero velocity
        self.node.sendV(0.0)
        self.assertEqual(self.node.last_twist.linear.x, 0.0)
        self.node.pub.publish.assert_called_once()

        # Reset mock
        self.node.pub.publish.reset_mock()

        # Test with positive velocity
        self.node.sendV(0.5)
        self.assertEqual(self.node.last_twist.linear.x, 0.5)
        self.node.pub.publish.assert_called_once()

        # Reset mock
        self.node.pub.publish.reset_mock()

        # Test with negative velocity
        self.node.sendV(-0.3)
        self.assertEqual(self.node.last_twist.linear.x, -0.3)
        self.node.pub.publish.assert_called_once()

    def test_send_w(self):
        """Test sending angular velocity"""
        # Test with zero angular velocity
        self.node.sendW(0.0)
        self.assertEqual(self.node.last_twist.angular.z, 0.0)
        self.node.pub.publish.assert_called_once()

        # Reset mock
        self.node.pub.publish.reset_mock()

        # Test with positive angular velocity
        self.node.sendW(1.0)
        self.assertEqual(self.node.last_twist.angular.z, 1.0)
        self.node.pub.publish.assert_called_once()

        # Reset mock
        self.node.pub.publish.reset_mock()

        # Test with negative angular velocity
        self.node.sendW(-0.7)
        self.assertEqual(self.node.last_twist.angular.z, -0.7)
        self.node.pub.publish.assert_called_once()

    def test_state_preservation(self):
        """Test that linear and angular velocities are preserved independently"""
        # Set initial values
        self.node.sendV(0.5)
        self.node.sendW(0.7)

        # Reset mock to clear call history
        self.node.pub.publish.reset_mock()

        # Only change linear velocity
        self.node.sendV(0.2)

        # Verify angular velocity is preserved
        self.assertEqual(self.node.last_twist.linear.x, 0.2)
        self.assertEqual(self.node.last_twist.angular.z, 0.7)

        # Only change angular velocity
        self.node.sendW(0.3)

        # Verify linear velocity is preserved
        self.assertEqual(self.node.last_twist.linear.x, 0.2)
        self.assertEqual(self.node.last_twist.angular.z, 0.3)

    def test_verify_published_message(self):
        """Test that the published message contains correct values"""
        # Set specific velocities
        self.node.sendV(0.4)
        self.node.sendW(0.6)

        # Get the last published message from the mock
        args, _ = self.node.pub.publish.call_args
        published_twist = args[0]

        # Verify published message content
        self.assertEqual(published_twist.linear.x, 0.4)
        self.assertEqual(published_twist.angular.z, 0.6)

        # Verify other fields are zero
        self.assertEqual(published_twist.linear.y, 0.0)
        self.assertEqual(published_twist.linear.z, 0.0)
        self.assertEqual(published_twist.angular.x, 0.0)
        self.assertEqual(published_twist.angular.y, 0.0)

    def test_multiple_commands(self):
        """Test sending multiple commands in sequence"""
        # Define a sequence of commands
        commands = [
            (0.1, 0.0),  # Move forward slowly
            (0.2, 0.5),  # Turn right while moving forward
            (0.0, 1.0),  # Rotate in place
            (-0.1, 0.0),  # Move backward slowly
            (0.0, 0.0),  # Stop
        ]

        # Execute each command
        for linear, angular in commands:
            self.node.pub.publish.reset_mock()

            # Send both velocities
            self.node.sendV(linear)
            self.node.sendW(angular)

            # Check internal state
            self.assertEqual(self.node.last_twist.linear.x, linear)
            self.assertEqual(self.node.last_twist.angular.z, angular)

            # Check that publish was called twice (once for each command)
            self.assertEqual(self.node.pub.publish.call_count, 2)


if __name__ == "__main__":
    unittest.main()
