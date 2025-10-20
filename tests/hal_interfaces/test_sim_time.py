import unittest
from unittest.mock import MagicMock, patch
import time

# Import the module to test
from hal_interfaces.general.sim_time import (
    SimTimeData,
    simTime2SimTimeData,
    SimTimeNode,
)


class MockClock:
    """Creates a mock Clock message with realistic data"""

    def __init__(self, sec=0, nanosec=0):
        """Initialize a mock Clock with specific time values"""
        self.clock = MagicMock()
        self.clock.sec = sec
        self.clock.nanosec = nanosec


class TestSimTimeData(unittest.TestCase):
    """Test the SimTimeData class"""

    def test_initialization(self):
        """Test that SimTimeData initializes with correct default values"""
        time_data = SimTimeData()
        self.assertEqual(time_data.seconds, 0)
        self.assertEqual(time_data.nanoseconds, 0)

    def test_string_representation(self):
        """Test the string representation of SimTimeData"""
        time_data = SimTimeData()
        time_data.seconds = 10
        time_data.nanoseconds = 500000000

        str_repr = str(time_data)
        self.assertIn("sec: 10", str_repr)
        self.assertIn("nanosec: 500000000", str_repr)


class TestSimTime2SimTimeData(unittest.TestCase):
    """Test the simTime2SimTimeData conversion function"""

    def test_basic_conversion(self):
        """Test basic conversion from Clock to SimTimeData"""
        # Create a mock clock message
        mock_clock = MockClock(sec=42, nanosec=123456789)

        # Convert to SimTimeData
        sim_time = simTime2SimTimeData(mock_clock)

        # Check that values were copied correctly
        self.assertEqual(sim_time.seconds, 42)
        self.assertEqual(sim_time.nanoseconds, 123456789)

    def test_zero_time(self):
        """Test conversion with zero time values"""
        mock_clock = MockClock(sec=0, nanosec=0)
        sim_time = simTime2SimTimeData(mock_clock)

        self.assertEqual(sim_time.seconds, 0)
        self.assertEqual(sim_time.nanoseconds, 0)

    def test_large_values(self):
        """Test with large time values"""
        # Simulate a simulation that's been running for days
        large_sec = 86400 * 5  # 5 days in seconds
        mock_clock = MockClock(sec=large_sec, nanosec=999999999)

        sim_time = simTime2SimTimeData(mock_clock)
        self.assertEqual(sim_time.seconds, large_sec)
        self.assertEqual(sim_time.nanoseconds, 999999999)


class TestSimTimeNode(unittest.TestCase):
    """Test the SimTimeNode class"""

    @patch("hal_interfaces.general.sim_time.Node.__init__")
    @patch("hal_interfaces.general.sim_time.QoSProfile")
    def test_node_initialization(self, mock_qos, mock_init):
        """Test SimTimeNode initialization and QoS setup"""
        # Avoid actual ROS2 initialization
        mock_init.return_value = None
        mock_qos.return_value = "mocked_qos_profile"

        # Mock the create_subscription method
        with patch(
            "hal_interfaces.general.sim_time.Node.create_subscription",
            return_value="mocked_subscription",
        ):

            # Initialize node
            node = SimTimeNode()

            # Check that Node was initialized with correct name
            mock_init.assert_called_once_with("simulation_time_node")

            # Check that QoS was configured correctly
            mock_qos.assert_called_once()

            # Check that subscription was created
            self.assertEqual(node.sub, "mocked_subscription")

            # Check that last_sim_time_ was initialized
            self.assertTrue(hasattr(node, "last_sim_time_"))

    @patch("hal_interfaces.general.sim_time.Node.__init__")
    def test_listener_callback(self, mock_init):
        """Test the listener_callback method"""
        # Avoid actual ROS2 initialization
        mock_init.return_value = None

        # Mock the create_subscription method
        with patch(
            "hal_interfaces.general.sim_time.Node.create_subscription",
            return_value=MagicMock(),
        ):

            node = SimTimeNode()

            # Create a mock clock message
            mock_clock = MockClock(sec=100, nanosec=200000000)

            # Call the callback
            node.listener_callback(mock_clock)

            # Check that last_sim_time_ was updated
            self.assertEqual(node.last_sim_time_, mock_clock)

    @patch("hal_interfaces.general.sim_time.Node.__init__")
    def test_get_sim_time(self, mock_init):
        """Test the getSimTime method"""
        # Avoid actual ROS2 initialization
        mock_init.return_value = None

        # Mock the create_subscription method
        with patch(
            "hal_interfaces.general.sim_time.Node.create_subscription",
            return_value=MagicMock(),
        ):

            node = SimTimeNode()

            # Create a mock clock message and set it as the last time
            mock_clock = MockClock(sec=50, nanosec=750000000)
            node.last_sim_time_ = mock_clock

            # Call getSimTime
            sim_time = node.getSimTime()

            # Check the result
            self.assertIsInstance(sim_time, SimTimeData)
            self.assertEqual(sim_time.seconds, 50)
            self.assertEqual(sim_time.nanoseconds, 750000000)

    @patch("hal_interfaces.general.sim_time.Node.__init__")
    def test_simulation_sequence(self, mock_init):
        """Test a sequence of simulation time updates"""
        # Avoid actual ROS2 initialization
        mock_init.return_value = None

        # Mock the create_subscription method
        with patch(
            "hal_interfaces.general.sim_time.Node.create_subscription",
            return_value=MagicMock(),
        ):

            node = SimTimeNode()

            # Simulate a sequence of time updates
            time_points = [
                (0, 0),  # Start of simulation
                (10, 0),  # 10 seconds later
                (10, 500000000),  # 10.5 seconds
                (60, 0),  # 1 minute
                (120, 750000000),  # 2 minutes, 0.75 seconds
            ]

            for sec, nanosec in time_points:
                # Create a mock clock message
                mock_clock = MockClock(sec=sec, nanosec=nanosec)

                # Call the callback
                node.listener_callback(mock_clock)

                # Get the time and verify
                sim_time = node.getSimTime()
                self.assertEqual(sim_time.seconds, sec)
                self.assertEqual(sim_time.nanoseconds, nanosec)


if __name__ == "__main__":
    unittest.main()
