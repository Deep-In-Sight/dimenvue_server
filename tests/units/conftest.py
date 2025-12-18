"""
Pytest configuration for unit tests.

Unit tests use REAL modules, not mocks.
"""

import pytest


def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line(
        "markers", "ros2: mark test as requiring ROS2 environment"
    )


@pytest.fixture(scope="session")
def ros2_env_check():
    """Check if ROS2 environment is available."""
    import subprocess
    try:
        result = subprocess.run(
            ["ros2", "--help"],
            capture_output=True,
            timeout=5
        )
        return result.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
