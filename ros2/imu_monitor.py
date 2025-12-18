"""
IMU Monitor Node for detecting sensor stabilization.

Uses moving standard deviation (mstd) over a 1-second window to detect
when the IMU is stable. This method provides better separation between
static and moving states compared to back-to-back sample comparison.
"""

from typing import Optional, Deque
from collections import deque
import time
import argparse
import os
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


class IMUMonitorNode(Node):
    """
    ROS2 node for monitoring IMU data and stabilization.
    Uses moving standard deviation to detect when IMU is stable.
    """

    def __init__(self, imu_topic: str, track_duration: float, result_path: str,
                 window_duration: float = 1.0,
                 accel_mstd_threshold: float = 0.3,
                 gyro_mstd_threshold: float = 0.05):
        super().__init__('imu_monitor')

        # Configuration
        self.track_duration = track_duration
        self.result_path = result_path
        self.window_duration = window_duration  # 1-second moving window

        # Thresholds for moving standard deviation
        self.accel_mstd_threshold = accel_mstd_threshold  # m/s^2
        self.gyro_mstd_threshold = gyro_mstd_threshold    # rad/s

        # Data buffers for moving window (stores tuples of (timestamp, value))
        self.accel_x_buffer: Deque[tuple] = deque()
        self.accel_y_buffer: Deque[tuple] = deque()
        self.accel_z_buffer: Deque[tuple] = deque()
        self.gyro_x_buffer: Deque[tuple] = deque()
        self.gyro_y_buffer: Deque[tuple] = deque()
        self.gyro_z_buffer: Deque[tuple] = deque()

        # Stabilization tracking
        self.tracking_start_time: Optional[float] = None
        self.is_first_message = True

        # Ensure result directory exists
        result_dir = os.path.dirname(self.result_path)
        if result_dir:
            Path(result_dir).mkdir(parents=True, exist_ok=True)
        # Delete old result file if exists
        if os.path.exists(self.result_path):
            os.remove(self.result_path)

        # QoS profile for IMU topic (sensor data typically uses BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to IMU topic
        self.subscription = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            qos_profile
        )

    def imu_callback(self, msg: Imu):
        """Callback for IMU messages."""
        current_time = time.time()

        if self.is_first_message:
            self.get_logger().info("IMU stabilization tracking started")
            self._write_status("TRACKING")
            self.is_first_message = False

        # Add current readings to buffers
        self._add_to_buffer(self.accel_x_buffer, current_time, msg.linear_acceleration.x)
        self._add_to_buffer(self.accel_y_buffer, current_time, msg.linear_acceleration.y)
        self._add_to_buffer(self.accel_z_buffer, current_time, msg.linear_acceleration.z)
        self._add_to_buffer(self.gyro_x_buffer, current_time, msg.angular_velocity.x)
        self._add_to_buffer(self.gyro_y_buffer, current_time, msg.angular_velocity.y)
        self._add_to_buffer(self.gyro_z_buffer, current_time, msg.angular_velocity.z)

        # Prune old data from buffers
        self._prune_buffer(self.accel_x_buffer, current_time)
        self._prune_buffer(self.accel_y_buffer, current_time)
        self._prune_buffer(self.accel_z_buffer, current_time)
        self._prune_buffer(self.gyro_x_buffer, current_time)
        self._prune_buffer(self.gyro_y_buffer, current_time)
        self._prune_buffer(self.gyro_z_buffer, current_time)

        # Need enough data for meaningful std calculation (at least 10 samples)
        if len(self.accel_x_buffer) < 10:
            return

        # Calculate moving standard deviations
        accel_mstd = max(
            self._calculate_std(self.accel_x_buffer),
            self._calculate_std(self.accel_y_buffer),
            self._calculate_std(self.accel_z_buffer)
        )
        gyro_mstd = max(
            self._calculate_std(self.gyro_x_buffer),
            self._calculate_std(self.gyro_y_buffer),
            self._calculate_std(self.gyro_z_buffer)
        )

        # Check if within thresholds
        is_stable = (accel_mstd <= self.accel_mstd_threshold and
                     gyro_mstd <= self.gyro_mstd_threshold)

        if is_stable:
            if self.tracking_start_time is None:
                # Start tracking stable duration
                self.tracking_start_time = current_time
            elif current_time - self.tracking_start_time >= self.track_duration:
                # Stable for required duration
                self._write_status("STABILIZED")
                self.get_logger().info("IMU stabilized, bye!")
                rclpy.shutdown()
        else:
            # Reset tracking if not stable
            self.tracking_start_time = None

    def _add_to_buffer(self, buffer: Deque, timestamp: float, value: float):
        """Add a timestamped value to the buffer."""
        buffer.append((timestamp, value))

    def _prune_buffer(self, buffer: Deque, current_time: float):
        """Remove data older than window_duration from the buffer."""
        cutoff_time = current_time - self.window_duration
        while buffer and buffer[0][0] < cutoff_time:
            buffer.popleft()

    def _calculate_std(self, buffer: Deque) -> float:
        """
        Calculate standard deviation of values in the buffer.

        Args:
            buffer: Deque of (timestamp, value) tuples

        Returns:
            Standard deviation of values, or 0.0 if buffer is empty
        """
        if len(buffer) < 2:
            return 0.0

        values = [v for _, v in buffer]
        n = len(values)
        mean = sum(values) / n
        variance = sum((x - mean) ** 2 for x in values) / n
        return math.sqrt(variance)

    def _write_status(self, status: str):
        """
        Write the current status to the result file.

        Args:
            status: Status string to write ("TRACKING" or "STABILIZED")
        """
        try:
            with open(self.result_path, 'w') as f:
                f.write(status)
        except Exception as e:
            self.get_logger().error(f"Failed to write status to {self.result_path}: {e}")


def main(args=None):
    """Main entry point for the IMU monitor node."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='IMU Monitor Node')
    parser.add_argument('--imu-topic', type=str, default='/livox/imu',
                        help='IMU topic to subscribe to (default: /livox/imu)')
    parser.add_argument('--track-duration', type=float, default=10.0,
                        help='Duration in seconds to track stability (default: 10.0)')
    parser.add_argument('--result-path', type=str, required=True,
                        help='Path to write the stabilization result (TRACKING/STABILIZED)')
    parser.add_argument('--window-duration', type=float, default=1.0,
                        help='Moving window duration in seconds (default: 1.0)')
    parser.add_argument('--accel-threshold', type=float, default=0.3,
                        help='Accelerometer mstd threshold in m/s^2 (default: 0.3)')
    parser.add_argument('--gyro-threshold', type=float, default=0.05,
                        help='Gyroscope mstd threshold in rad/s (default: 0.05)')

    parsed_args = parser.parse_args(args)

    rclpy.init()

    imu_node = IMUMonitorNode(
        imu_topic=parsed_args.imu_topic,
        track_duration=parsed_args.track_duration,
        result_path=parsed_args.result_path,
        window_duration=parsed_args.window_duration,
        accel_mstd_threshold=parsed_args.accel_threshold,
        gyro_mstd_threshold=parsed_args.gyro_threshold
    )

    try:
        rclpy.spin(imu_node)
    finally:
        # Destroy the node explicitly
        imu_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
