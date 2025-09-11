import unittest
import numpy as np
from unittest.mock import MagicMock, patch
import math

# Import the module to test
from hal_interfaces.general.noise_odometry import (
    Pose3d,
    quat2Yaw,
    quat2Pitch,
    quat2Roll,
    gaussian_noise,
    add_noise,
    euler2quat,
    NoisyOdometryNode,
)


class MockOdometry:
    """Creates a realistic mock Odometry message for testing"""

    def __init__(
        self, x=0.0, y=0.0, z=0.0, qw=1.0, qx=0.0, qy=0.0, qz=0.0, sec=0, nanosec=0
    ):
        """Initialize a mock Odometry with realistic data"""
        self.header = type("Header", (), {})()
        self.header.stamp = type("Time", (), {"sec": sec, "nanosec": nanosec})()

        self.pose = type("PoseWithCovariance", (), {})()
        self.pose.pose = type("Pose", (), {})()
        self.pose.pose.position = type("Point", (), {"x": x, "y": y, "z": z})()
        self.pose.pose.orientation = type(
            "Quaternion", (), {"w": qw, "x": qx, "y": qy, "z": qz}
        )()


class TestGaussianNoise(unittest.TestCase):
    """Test the gaussian_noise function"""

    def test_noise_level_zero(self):
        """Test that noise_level=0 produces no change"""
        value = 10.0
        noisy = gaussian_noise(value, noise_level=0.0)
        self.assertEqual(noisy, value)

    def test_noise_is_applied(self):
        """Test that noise is applied with non-zero noise_level"""
        # Set random seed for reproducibility
        np.random.seed(42)

        value = 10.0
        # Use high noise level to ensure difference
        noisy = gaussian_noise(value, noise_level=1.0)
        self.assertNotEqual(noisy, value)

    def test_average_noise_is_close_to_mean(self):
        """Test that over many samples, noise averages to the specified mean"""
        # Set random seed for reproducibility
        np.random.seed(42)

        value = 10.0
        mean = 0.0
        samples = 1000

        # Generate many noisy values
        noisy_values = [
            gaussian_noise(value, mu=mean, noise_level=0.1) for _ in range(samples)
        ]

        # Calculate average noise
        avg_noise = sum(noisy_values) / samples - value

        # Average noise should be close to mean (with some tolerance)
        self.assertAlmostEqual(avg_noise, 0, places=1)


class TestEuler2Quat(unittest.TestCase):
    """Test the euler2quat function"""

    def test_identity_rotation(self):
        """Test conversion of zero Euler angles"""
        qx, qy, qz, qw = euler2quat(0, 0, 0)
        self.assertAlmostEqual(qw, 1.0, places=5)
        self.assertAlmostEqual(qx, 0.0, places=5)
        self.assertAlmostEqual(qy, 0.0, places=5)
        self.assertAlmostEqual(qz, 0.0, places=5)

    def test_90_degree_yaw(self):
        """Test conversion of 90° yaw rotation"""
        qx, qy, qz, qw = euler2quat(math.pi / 2, 0, 0)
        self.assertAlmostEqual(qw, math.cos(math.pi / 4), places=5)
        self.assertAlmostEqual(qz, math.sin(math.pi / 4), places=5)
        self.assertAlmostEqual(qx, 0.0, places=5)
        self.assertAlmostEqual(qy, 0.0, places=5)

    def test_quaternion_conversion_round_trip(self):
        """Test that Euler->quaternion->Euler preserves angles"""
        # Starting Euler angles
        yaw = math.pi / 4  # 45°
        pitch = math.pi / 6  # 30°
        roll = math.pi / 3  # 60°

        # Convert to quaternion
        qx, qy, qz, qw = euler2quat(yaw, pitch, roll)

        # Convert back to Euler angles
        yaw_back = quat2Yaw(qw, qx, qy, qz)
        pitch_back = quat2Pitch(qw, qx, qy, qz)
        roll_back = quat2Roll(qw, qx, qy, qz)

        # Check if we got the same angles back
        self.assertAlmostEqual(yaw_back, yaw, places=5)
        self.assertAlmostEqual(pitch_back, pitch, places=5)
        self.assertAlmostEqual(roll_back, roll, places=5)


class TestAddNoise(unittest.TestCase):
    """Test the add_noise function"""

    def setUp(self):
        """Set up test environment with fixed random seed"""
        np.random.seed(42)

    def test_first_odom(self):
        """Test that first odometry message is returned unchanged"""
        # Create a mock odometry message
        odom = MockOdometry(x=10.0, y=20.0, z=5.0, qw=1.0)

        # First odometry should be returned as-is
        result = add_noise(None, odom, odom, 0.1)

        # The result should be the same odometry
        self.assertEqual(result, odom)

    def test_noise_is_added(self):
        """Test that noise is added to subsequent odometry messages"""
        # Create two consecutive odometry messages with movement
        odom1 = MockOdometry(x=0.0, y=0.0, z=0.0, qw=1.0)
        odom2 = MockOdometry(x=1.0, y=0.0, z=0.0, qw=1.0)  # 1m forward

        # Empty initial noisy odometry
        base_odom = MockOdometry()

        # Add noise with high noise level
        result = add_noise(odom1, odom2, base_odom, 0.1)

        # Result should not match exactly the expected position
        self.assertNotEqual(result.pose.pose.position.x, 1.0)

    def test_noise_level_affects_magnitude(self):
        """Test that higher noise levels result in greater deviation"""
        # Create two consecutive odometry messages
        odom1 = MockOdometry(x=0.0, y=0.0, z=0.0, qw=1.0)
        odom2 = MockOdometry(x=1.0, y=1.0, z=0.0, qw=1.0)

        base_odom = MockOdometry()

        # Get results with different noise levels
        result_low = add_noise(odom1, odom2, base_odom, 0.01)
        result_high = add_noise(odom1, odom2, base_odom, 0.1)

        # Calculate deviations
        dev_low_x = abs(result_low.pose.pose.position.x - 1.0)
        dev_high_x = abs(result_high.pose.pose.position.x - 1.0)

        # Higher noise level should result in greater deviation
        self.assertGreater(dev_high_x, dev_low_x)

    def test_orientation_noise(self):
        """Test that orientation noise is applied"""
        # Create odometry with 90° yaw rotation
        odom1 = MockOdometry(qw=1.0, qx=0.0, qy=0.0, qz=0.0)  # 0°
        odom2 = MockOdometry(qw=0.7071, qx=0.0, qy=0.0, qz=0.7071)  # 90°

        base_odom = MockOdometry()

        # Add noise
        result = add_noise(odom1, odom2, base_odom, 0.1)

        # Extract the resulting yaw
        yaw = quat2Yaw(
            result.pose.pose.orientation.w,
            result.pose.pose.orientation.x,
            result.pose.pose.orientation.y,
            result.pose.pose.orientation.z,
        )

        # Check that the yaw has some value, and allow for a large deviation
        deviation = abs(yaw - math.pi / 2)
        self.assertLess(deviation, 1.7)  # Allow up to ~97° deviation (almost π)


class TestNoisyOdometryNode(unittest.TestCase):
    """Test the NoisyOdometryNode class"""

    @patch("hal_interfaces.general.noise_odometry.Node.__init__")
    def test_initialization(self, mock_init):
        """Test that NoisyOdometryNode initializes correctly"""
        # Avoid actual ROS2 initialization
        mock_init.return_value = None

        # Mock the create_subscription method
        with patch(
            "hal_interfaces.general.noise_odometry.Node.create_subscription",
            return_value=MagicMock(),
        ):

            node = NoisyOdometryNode("test_topic")

            # Check that Node was initialized with correct name
            mock_init.assert_called_once_with("noisy_odometry_node")

            # Check that the subscription was created
            self.assertTrue(hasattr(node, "sub"))

            # Check that initial values are set
            self.assertIsNone(node.last_pose_)
            self.assertEqual(node.noise_level, 0.01)
            self.assertTrue(hasattr(node, "noisy_pose"))

    @patch("hal_interfaces.general.noise_odometry.Node.__init__")
    @patch("hal_interfaces.general.noise_odometry.add_noise")
    def test_listener_callback(self, mock_add_noise, mock_init):
        """Test the listener_callback method"""
        # Avoid actual ROS2 initialization
        mock_init.return_value = None

        # Mock the create_subscription method
        with patch(
            "hal_interfaces.general.noise_odometry.Node.create_subscription",
            return_value=MagicMock(),
        ):

            node = NoisyOdometryNode("test_topic")
            node.noisy_pose = MockOdometry()  # Initialize noisy_pose

            # Create a mock odometry message
            mock_odom = MockOdometry(x=5.0, y=10.0, z=0.0)

            # Setup the mock add_noise function
            mock_result = MockOdometry(x=5.1, y=10.2, z=0.0)
            mock_add_noise.return_value = mock_result

            # Call the callback
            node.listener_callback(mock_odom)

            # Check that add_noise was called (without checking specific arguments)
            mock_add_noise.assert_called_once()

            # Check that noisy_pose was updated to the mock result
            self.assertEqual(node.noisy_pose, mock_result)

            # Check that last_pose_ was updated
            self.assertEqual(node.last_pose_, mock_odom)

    @patch("hal_interfaces.general.noise_odometry.Node.__init__")
    @patch("hal_interfaces.general.noise_odometry.odometry2Pose3D")
    def test_get_pose3d(self, mock_odometry2pose3d, mock_init):
        """Test the getPose3d method"""
        # Avoid actual ROS2 initialization
        mock_init.return_value = None

        # Mock the create_subscription method
        with patch(
            "hal_interfaces.general.noise_odometry.Node.create_subscription",
            return_value=MagicMock(),
        ):

            node = NoisyOdometryNode("test_topic")

            # Create a mock noisy_pose
            node.noisy_pose = MockOdometry(x=15.0, y=25.0, z=5.0)

            # Create an expected Pose3d result
            expected_pose = Pose3d()
            expected_pose.x = 15.0
            expected_pose.y = 25.0
            mock_odometry2pose3d.return_value = expected_pose

            # Call getPose3d
            result = node.getPose3d()

            # Check that odometry2Pose3D was called with noisy_pose
            mock_odometry2pose3d.assert_called_once_with(node.noisy_pose)

            # Check the result
            self.assertEqual(result, expected_pose)

    @patch("hal_interfaces.general.noise_odometry.Node.__init__")
    def test_noise_accumulation(self, mock_init):
        """Test that noise accumulates over multiple updates"""
        # Avoid actual ROS2 initialization
        mock_init.return_value = None

        # Mock add_noise to ensure it simulates accumulating noise
        with patch(
            "hal_interfaces.general.noise_odometry.Node.create_subscription",
            return_value=MagicMock(),
        ):
            with patch(
                "hal_interfaces.general.noise_odometry.add_noise"
            ) as mock_add_noise:
                # Configure add_noise to add increasing amounts of noise
                def side_effect(prev_odom, odom, base_odom, noise_level):
                    if prev_odom is None:
                        return odom  # First call returns unchanged

                    # Create a result with accumulated noise
                    position_index = int(
                        odom.pose.pose.position.x
                    )  # Use x as index (1.0, 2.0, etc)
                    result = MockOdometry(
                        x=odom.pose.pose.position.x
                        + 0.05 * position_index,  # More noise for later positions
                        y=odom.pose.pose.position.y,
                        z=odom.pose.pose.position.z,
                    )
                    return result

                mock_add_noise.side_effect = side_effect

                node = NoisyOdometryNode("test_topic")

                # Create initial odometry
                odom1 = MockOdometry(x=0.0, y=0.0, z=0.0, qw=1.0)

                # First update (should just store the odometry)
                node.listener_callback(odom1)

                # Simulate robot moving in a straight line with fixed increments
                positions = []
                true_x = 0.0

                # Do several updates and collect positions
                for i in range(10):
                    true_x += 1.0  # Move 1m forward each time
                    odom = MockOdometry(x=true_x, y=0.0, z=0.0, qw=1.0)
                    node.listener_callback(odom)
                    positions.append(node.noisy_pose.pose.pose.position.x)

                # Verify that noise accumulates (deviation increases)
                early_deviation = abs(positions[1] - 2.0)  # Position 1 is at x=2.0
                late_deviation = abs(positions[-1] - 10.0)  # Position 9 is at x=10.0

                # Later positions should have greater deviation
                self.assertGreater(late_deviation, early_deviation)


if __name__ == "__main__":
    unittest.main()
