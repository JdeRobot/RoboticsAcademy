import unittest
from unittest.mock import MagicMock, patch
import threading
import time
import json
from gui_interfaces.general.threading_gui import ThreadingGUI

class TestThreadingGUI(unittest.TestCase):
    @patch('rclpy.create_node')
    @patch('rclpy.init')
    @patch('websocket.WebSocketApp')
    def setUp(self, mock_websocket, mock_init, mock_create_node):
        # Mock ROS and websocket initialization
        self.mock_websocket = mock_websocket
        self.gui = ThreadingGUI(host="ws://test:2303", freq=10.0)
        
        # Mock client to avoid actual websocket connection
        self.gui.client = MagicMock()
        
        # Override the update_gui method for testing
        self.gui.update_gui = MagicMock()
        
    def test_initialization(self):
        # Test initialization parameters
        self.assertEqual(self.gui.host, "ws://test:2303")
        self.assertEqual(self.gui.out_period, 0.1)  # 1/10Hz
        self.assertTrue(self.gui.ack)
        self.assertFalse(self.gui.ack_frontend)
        
    def test_gui_in_thread_ack_message(self):
        # Test handling of ack message
        self.gui.gui_in_thread(None, "ack")
        
        # Verify ack flags are set correctly
        self.assertTrue(self.gui.ack)
        self.assertTrue(self.gui.ack_frontend)
        
    def test_gui_in_thread_unsupported_message(self):
        # Test handling of unsupported message
        with patch('manager.ram_logging.log_manager.LogManager.logger.error') as mock_error:
            self.gui.gui_in_thread(None, "unsupported")
            mock_error.assert_called_once_with("Unsupported msg")
            
    def test_send_to_client(self):
        # Test sending message to client
        self.gui.send_to_client("test_message")
        self.gui.client.send.assert_called_once_with("test_message")
        
    def test_send_to_client_exception(self):
        # Test exception handling when sending message
        self.gui.client.send.side_effect = Exception("Test exception")
        
        with patch('manager.ram_logging.log_manager.LogManager.logger.info') as mock_info:
            self.gui.send_to_client("test_message")
            mock_info.assert_called_once()
            self.assertIn("Error sending message", mock_info.call_args[0][0])
            
    @patch('threading.Thread')
    def test_start_method(self, mock_thread):
        # Test that start method creates the required threads
        self.gui.start()
        
        # Verify that two threads were created
        self.assertEqual(mock_thread.call_count, 2)
        
        # Verify thread targets
        thread_targets = [call_args[1]['target'] for call_args in mock_thread.call_args_list]
        self.assertIn(self.gui.run_websocket, thread_targets)
        self.assertIn(self.gui.gui_out_thread, thread_targets)
