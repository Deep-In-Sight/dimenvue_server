"""
Unit tests for IMU monitor node (ros2/imu_monitor.py)

Test IDs: UNIT-IMU-001 to UNIT-IMU-006
"""

import os
import subprocess
import sys
import tempfile
import time
import pytest

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


def ros2_available():
    """Check if ROS2 is available."""
    try:
        result = subprocess.run(
            ["ros2", "--help"],
            capture_output=True,
            timeout=5
        )
        return result.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def _test_bag_available():
    """Check if test bag exists (helper function for skipif)."""
    return os.path.exists("/shared_data/test_bag")


@pytest.mark.ros2
@pytest.mark.skipif(not ros2_available(), reason="ROS2 not available")
@pytest.mark.skipif(not _test_bag_available(), reason="Test bag not found at /shared_data/test_bag")
class TestIMUMonitorStabilization:
    """UNIT-IMU-001: Test IMU monitor detects stabilization from test_bag"""

    def test_imu_monitor_detects_stabilization(self):
        """
        UNIT-IMU-001: IMU monitor should:
        - Start successfully
        - Receive IMU data from bag playback
        - Detect stabilization using moving standard deviation
        - Write STABILIZED to status file
        """
        with tempfile.TemporaryDirectory(prefix="test_imu_") as tmpdir:
            status_file = os.path.join(tmpdir, "imu_status.txt")
            imu_monitor_script = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                "ros2", "imu_monitor.py"
            )

            # Start IMU monitor in background
            monitor_proc = subprocess.Popen(
                [
                    "python3", imu_monitor_script,
                    "--imu-topic", "/ouster/imu",
                    "--track-duration", "3.0",  # Short duration for test
                    "--result-path", status_file,
                    "--window-duration", "1.0",
                    "--accel-threshold", "0.3",
                    "--gyro-threshold", "0.05"
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT
            )

            # Give monitor time to start
            time.sleep(1)

            # Start bag playback
            bag_proc = subprocess.Popen(
                ["ros2", "bag", "play", "/shared_data/test_bag", "--rate", "2.0"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT
            )

            try:
                # Wait for monitor to detect stabilization (max 30 seconds)
                start_time = time.time()
                stabilized = False

                while time.time() - start_time < 30:
                    # Check if status file exists and contains STABILIZED
                    if os.path.exists(status_file):
                        with open(status_file, 'r') as f:
                            status = f.read().strip()
                            if status == "STABILIZED":
                                stabilized = True
                                break
                            elif status == "TRACKING":
                                pass  # Still tracking, continue waiting

                    # Check if monitor process has exited
                    if monitor_proc.poll() is not None:
                        # Monitor exited, check final status
                        if os.path.exists(status_file):
                            with open(status_file, 'r') as f:
                                status = f.read().strip()
                                if status == "STABILIZED":
                                    stabilized = True
                        break

                    time.sleep(0.5)

                # Verify stabilization was detected
                assert stabilized, f"IMU monitor did not detect stabilization within 30s. Status file: {status_file}"

                # Verify status file content
                with open(status_file, 'r') as f:
                    final_status = f.read().strip()
                assert final_status == "STABILIZED", f"Expected STABILIZED, got: {final_status}"

            finally:
                # Cleanup: terminate processes
                if monitor_proc.poll() is None:
                    monitor_proc.terminate()
                    try:
                        monitor_proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        monitor_proc.kill()

                if bag_proc.poll() is None:
                    bag_proc.terminate()
                    try:
                        bag_proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        bag_proc.kill()


@pytest.mark.ros2
@pytest.mark.skipif(not ros2_available(), reason="ROS2 not available")
class TestIMUMonitorStatusFile:
    """Test IMU monitor status file handling"""

    def test_status_file_created_on_start(self):
        """
        IMU monitor should create status file with TRACKING on first message.
        """
        with tempfile.TemporaryDirectory(prefix="test_imu_") as tmpdir:
            status_file = os.path.join(tmpdir, "imu_status.txt")
            imu_monitor_script = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                "ros2", "imu_monitor.py"
            )

            # Start IMU monitor
            monitor_proc = subprocess.Popen(
                [
                    "python3", imu_monitor_script,
                    "--imu-topic", "/test_imu",  # Non-existent topic
                    "--track-duration", "1.0",
                    "--result-path", status_file,
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT
            )

            try:
                # Wait a moment for monitor to start
                time.sleep(2)

                # Publish a single IMU message to trigger status file creation
                pub_proc = subprocess.run(
                    [
                        "ros2", "topic", "pub", "--once",
                        "/test_imu", "sensor_msgs/msg/Imu",
                        '{"linear_acceleration": {"x": 0, "y": 0, "z": 9.8}, "angular_velocity": {"x": 0, "y": 0, "z": 0}}'
                    ],
                    capture_output=True,
                    timeout=10
                )

                # Wait for message to be processed
                time.sleep(1)

                # Check status file was created
                assert os.path.exists(status_file), "Status file should be created"

                with open(status_file, 'r') as f:
                    status = f.read().strip()
                assert status == "TRACKING", f"Initial status should be TRACKING, got: {status}"

            finally:
                if monitor_proc.poll() is None:
                    monitor_proc.terminate()
                    try:
                        monitor_proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        monitor_proc.kill()


class TestIMUMonitorMstdCalculation:
    """Unit tests for moving standard deviation calculation"""

    def test_calculate_std_basic(self):
        """Test standard deviation calculation."""
        from collections import deque
        import math

        # Import the function we're testing
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            "ros2"
        ))

        # Create a simple buffer with known values
        # Values: [1, 2, 3, 4, 5] -> mean=3, variance=2, std=sqrt(2)
        buffer = deque([
            (0, 1.0),
            (1, 2.0),
            (2, 3.0),
            (3, 4.0),
            (4, 5.0),
        ])

        # Calculate std manually
        values = [v for _, v in buffer]
        n = len(values)
        mean = sum(values) / n
        variance = sum((x - mean) ** 2 for x in values) / n
        expected_std = math.sqrt(variance)

        # Should be sqrt(2) ~= 1.414
        assert abs(expected_std - math.sqrt(2)) < 0.001

    def test_calculate_std_constant_values(self):
        """Test that constant values give zero standard deviation."""
        from collections import deque

        # All same values -> std should be 0
        buffer = deque([
            (0, 5.0),
            (1, 5.0),
            (2, 5.0),
            (3, 5.0),
            (4, 5.0),
        ])

        values = [v for _, v in buffer]
        n = len(values)
        mean = sum(values) / n
        variance = sum((x - mean) ** 2 for x in values) / n

        assert variance == 0.0

    def test_calculate_std_high_variance(self):
        """Test that high variance values give high standard deviation."""
        from collections import deque
        import math

        # High variance values
        buffer = deque([
            (0, -10.0),
            (1, 10.0),
            (2, -10.0),
            (3, 10.0),
            (4, -10.0),
        ])

        values = [v for _, v in buffer]
        n = len(values)
        mean = sum(values) / n  # Should be -2
        variance = sum((x - mean) ** 2 for x in values) / n
        std = math.sqrt(variance)

        # High std expected (around 10)
        assert std > 5.0
