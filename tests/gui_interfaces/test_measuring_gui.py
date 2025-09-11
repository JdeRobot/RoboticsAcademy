import unittest
from unittest.mock import MagicMock, patch
import time
import json
from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI


class TestMeasuringThreadingGUI(unittest.TestCase):
    @patch("rclpy.create_node")
    @patch("rclpy.init")
    @patch("websocket.WebSocketApp")
    def setUp(self, mock_websocket, mock_init, mock_create_node):
        # Mock ROS and websocket initialization
        self.mock_websocket = mock_websocket
        self.gui = MeasuringThreadingGUI(host="ws://test:2303", freq=10.0)

        # Mock client to avoid actual websocket connection
        self.gui.client = MagicMock()

        # Override the update_gui method for testing
        self.gui.update_gui = MagicMock()

    def test_initialization(self):
        # Test initialization parameters
        self.assertEqual(self.gui.host, "ws://test:2303")
        self.assertEqual(self.gui.out_period, 0.1)  # 1/10Hz
        self.assertEqual(self.gui.ideal_cycle, 0.1)
        self.assertEqual(self.gui.iteration_counter, 0)

    def test_measure_thread(self):
        # Test the measure_thread method
        self.gui.iteration_counter = 10

        # Mock time.time() to return controlled values
        with patch("time.time") as mock_time:
            # First call to get start_time
            mock_time.return_value = 100.0
            self.gui.measure_thread()

            # Second call after 1 second (simulating 1 second passing)
            mock_time.return_value = 101.0
            self.gui.measure_thread()

            # Verify frequency calculation (10 iterations in 1 second = 10Hz)
            self.assertEqual(self.gui.real_freq, 10.0)

            # Reset counter for next measurement
            self.assertEqual(self.gui.iteration_counter, 0)

    def test_send_frequency_message(self):
        # Test sending frequency message to client
        self.gui.real_freq = 9.5  # Slightly lower than target
        self.gui.ideal_cycle = 0.1  # 10Hz target

        # Call method to send frequency info
        self.gui.send_frequency_message()

        # Verify message was sent with correct format
        self.gui.client.send.assert_called_once()
        message = self.gui.client.send.call_args[0][0]

        # Message should be JSON with frequency data
        self.assertTrue(message.startswith("#freq"))
        freq_data = json.loads(message[5:])  # Skip '#freq' prefix
        self.assertAlmostEqual(freq_data["real"], 9.5)
        self.assertAlmostEqual(freq_data["ideal"], 10.0)
