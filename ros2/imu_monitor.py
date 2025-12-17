from typing import Optional
import time
import argparse
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

class IMUMonitorNode(Node):
    """
    ROS2 node for monitoring IMU data and stabilization.
    Trigger a callback when IMU data has been stable for a specified duration.
    """

    def __init__(self, imu_topic: str, track_duration: float, result_path: str):
        super().__init__('imu_monitor')

        # Stabilization tracking variables
        self.previous_imu_data: Optional[Imu] = None
        self.tracking_start_time: Optional[float] = None
        self.track_duration = track_duration
        self.result_path = result_path
        
        # Ensure result directory exists
        result_dir = os.path.dirname(self.result_path)
        if result_dir:
            Path(result_dir).mkdir(parents=True, exist_ok=True)
        # delete old result file if exists
        if os.path.exists(self.result_path):
            os.remove(self.result_path)

        # Thresholds for stability detection
        self.accel_threshold = 0.1  # m/s^2 - threshold for acceleration fluctuation
        self.gyro_threshold = 0.05  # rad/s - threshold for angular velocity fluctuation

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
        
        # Check stability if we have previous data
        if self.previous_imu_data is None:
            self.get_logger().info("IMU stabilization tracking started")
            self._write_status("TRACKING")
        else:
            is_within_threshold = self._check_fluctuation_threshold(self.previous_imu_data, msg)
            
            if is_within_threshold:
                if self.tracking_start_time is None:
                    # Start tracking
                    self.tracking_start_time = current_time
                elif current_time - self.tracking_start_time >= self.track_duration:
                    self.stabilized = True
                    self._write_status("STABILIZED")
                    self.get_logger().info("IMU stabilized, bye!")
                    rclpy.shutdown()
            else:
                # reset
                self.tracking_start_time = None
        
        # Store current reading as previous for next comparison
        self.previous_imu_data = msg

    def _check_fluctuation_threshold(self, prev_msg: Imu, curr_msg: Imu) -> bool:
        """
        Check if fluctuation between two IMU readings is within threshold.
        
        Args:
            prev_msg: Previous IMU message
            curr_msg: Current IMU message
            
        Returns:
            True if fluctuation is within threshold, False otherwise
        """
        # Calculate acceleration magnitude differences
        accel_diff_x = abs(curr_msg.linear_acceleration.x - prev_msg.linear_acceleration.x)
        accel_diff_y = abs(curr_msg.linear_acceleration.y - prev_msg.linear_acceleration.y)
        accel_diff_z = abs(curr_msg.linear_acceleration.z - prev_msg.linear_acceleration.z)
        
        # Calculate gyroscope magnitude differences  
        gyro_diff_x = abs(curr_msg.angular_velocity.x - prev_msg.angular_velocity.x)
        gyro_diff_y = abs(curr_msg.angular_velocity.y - prev_msg.angular_velocity.y)
        gyro_diff_z = abs(curr_msg.angular_velocity.z - prev_msg.angular_velocity.z)
        
        # Check if all differences are within thresholds
        accel_stable = (accel_diff_x <= self.accel_threshold and 
                       accel_diff_y <= self.accel_threshold and 
                       accel_diff_z <= self.accel_threshold)
        
        gyro_stable = (gyro_diff_x <= self.gyro_threshold and 
                      gyro_diff_y <= self.gyro_threshold and 
                      gyro_diff_z <= self.gyro_threshold)
        
        return accel_stable and gyro_stable

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
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='IMU Monitor Node')
    parser.add_argument('--imu-topic', type=str, default='/livox/imu',
                        help='IMU topic to subscribe to (default: /livox/imu)')
    parser.add_argument('--track-duration', type=float, default=10.0,
                        help='Duration in seconds to track stability (default: 10.0)')
    parser.add_argument('--result-path', type=str, required=True,
                        help='Path to write the stabilization result (TRACKING/STABILIZED)')
    
    parsed_args = parser.parse_args(args)
    
    rclpy.init()

    imu_node = IMUMonitorNode(
        imu_topic=parsed_args.imu_topic,
        track_duration=parsed_args.track_duration,
        result_path=parsed_args.result_path
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