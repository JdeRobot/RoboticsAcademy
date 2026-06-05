import unittest
from unittest.mock import patch, mock_open, MagicMock
import os
import sys
from console_interfaces.general.console import start_console, close_console


class TestConsoleInterface(unittest.TestCase):
    @patch("os.listdir")
    @patch("builtins.open", new_callable=mock_open)
    def test_start_console(self, mock_file_open, mock_listdir):
        # Mock os.listdir to return a list of file descriptors
        mock_listdir.return_value = ["0", "1", "2", "3", "ptmx"]

        # Call the function to test
        start_console()

        # Verify that os.listdir was called with correct path
        mock_listdir.assert_called_once_with("/dev/pts/")

        # Verify that open was called 3 times (for stderr, stdout, stdin)
        self.assertEqual(mock_file_open.call_count, 3)

        # Verify that open was called with the correct path (using max fd = 3)
        expected_path = "/dev/pts/3"
        mock_file_open.assert_any_call(expected_path, "w")

        # Verify that sys streams were reassigned
        self.assertEqual(sys.stderr, mock_file_open())
        self.assertEqual(sys.stdout, mock_file_open())
        self.assertEqual(sys.stdin, mock_file_open())

    def test_close_console(self):
        # Mock sys streams
        sys.stderr = MagicMock()
        sys.stdout = MagicMock()
        sys.stdin = MagicMock()

        # Call the function to test
        close_console()

        # Verify that close was called on each stream
        sys.stderr.close.assert_called_once()
        sys.stdout.close.assert_called_once()
        sys.stdin.close.assert_called_once()
