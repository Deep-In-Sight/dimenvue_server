"""
EE Test Configuration - Uses REAL components (no mocks)

The pytest.ini in this directory makes it the rootdir, so the parent
tests/conftest.py (which mocks modules) is NOT loaded.
"""


def pytest_configure(config):
    """Register EE markers."""
    config.addinivalue_line("markers", "ee: End-to-end tests with real components")
    config.addinivalue_line("markers", "ros2: Tests requiring ROS2 environment")
    config.addinivalue_line("markers", "slow: Slow running tests")
