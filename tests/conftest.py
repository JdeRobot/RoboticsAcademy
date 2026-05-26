import pytest
import os
import sys
import django
from unittest.mock import MagicMock

# Add project root to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# Configure Django settings for testing
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "academy.settings.test")
django.setup()

# ROS environment setup
os.environ["ROS_VERSION"] = "2"

# Mock fixtures for ROS
@pytest.fixture
def mock_ros_node():
    from tests.mocks.mock_ros import MockROSNode

    return MockROSNode("test_node")


@pytest.fixture
def mock_ros_image():
    from tests.mocks.mock_ros_messages import MockROSImage

    return MockROSImage()


@pytest.fixture
def mock_ros_laser_scan():
    from tests.mocks.mock_ros_messages import MockROSLaserScan

    return MockROSLaserScan()


@pytest.fixture
def mock_websocket():
    from tests.mocks.mock_websocket import MockWebSocketApp

    return MockWebSocketApp("ws://test:2303")
