import pytest
import os
import sys
import django

# Add project root and test stubs folder to path so lightweight stubs are
# imported before any external packages during pytest collection.
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
STUBS_DIR = os.path.join(REPO_ROOT, "tests", "_stubs")
sys.path.insert(0, REPO_ROOT)
if os.path.isdir(STUBS_DIR):
    sys.path.insert(0, STUBS_DIR)

# Configure Django settings for testing (uses academy/test_settings.py)
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "academy.test_settings")
django.setup()

# ROS environment setup used by stubs
os.environ.setdefault("ROS_VERSION", "2")

# Lightweight mock fixtures can be added here if needed by tests. We avoid
# importing heavyweight or project-specific mock modules at collection time.
