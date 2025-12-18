#!/usr/bin/env python3
"""
Analyze IMU data from a rosbag to determine appropriate stability thresholds.

Usage:
    python3 analyze_imu.py /shared_data/test_bag --topic /ouster/imu
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

import rclpy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions


def read_imu_from_bag(bag_path: str, topic: str = "/ouster/imu"):
    """Read all IMU messages from a rosbag."""
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Get topic types
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    if topic not in type_map:
        print(f"Topic {topic} not found in bag. Available topics:")
        for t in topic_types:
            print(f"  {t.name}: {t.type}")
        return None

    timestamps = []
    accel_x, accel_y, accel_z = [], [], []
    gyro_x, gyro_y, gyro_z = [], [], []

    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, Imu)
            timestamps.append(timestamp / 1e9)  # Convert to seconds
            accel_x.append(msg.linear_acceleration.x)
            accel_y.append(msg.linear_acceleration.y)
            accel_z.append(msg.linear_acceleration.z)
            gyro_x.append(msg.angular_velocity.x)
            gyro_y.append(msg.angular_velocity.y)
            gyro_z.append(msg.angular_velocity.z)

    return {
        'timestamps': np.array(timestamps),
        'accel_x': np.array(accel_x),
        'accel_y': np.array(accel_y),
        'accel_z': np.array(accel_z),
        'gyro_x': np.array(gyro_x),
        'gyro_y': np.array(gyro_y),
        'gyro_z': np.array(gyro_z),
    }


def analyze_moving_std_stats(data, duration=None):
    """Analyze moving standard deviation statistics."""
    # If duration is specified, filter data to only analyze first N seconds
    if duration is not None:
        start_time = data['timestamps'][0]
        end_time = start_time + duration
        mask = data['timestamps'] <= end_time
        
        # Create filtered data for analysis
        filtered_data = {}
        for key in data.keys():
            filtered_data[key] = data[key][mask]
        
        print(f"Analyzing first {duration} seconds ({np.sum(mask)} out of {len(data['timestamps'])} messages)")
        analysis_data = filtered_data
    else:
        print("Analyzing all data")
        analysis_data = data

    # Calculate moving standard deviations
    results = {}
    for key in ['accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']:
        moving_std = calculate_moving_std(analysis_data[key], analysis_data['timestamps'], window_duration=1.0)
        results[key] = {
            'min': np.min(moving_std),
            'max': np.max(moving_std),
            'mean': np.mean(moving_std),
            'std': np.std(moving_std),
            'p50': np.percentile(moving_std, 50),
            'p90': np.percentile(moving_std, 90),
            'p95': np.percentile(moving_std, 95),
            'p99': np.percentile(moving_std, 99),
        }

    return results, analysis_data if duration is not None else None


def calculate_moving_std(values, timestamps, window_duration=1.0):
    """Calculate moving standard deviation with specified window duration."""
    dt = np.median(np.diff(timestamps))
    window_size = int(window_duration / dt)
    
    moving_std = np.zeros_like(values)
    for i in range(len(values)):
        start_idx = max(0, i - window_size // 2)
        end_idx = min(len(values), i + window_size // 2 + 1)
        moving_std[i] = np.std(values[start_idx:end_idx])
    
    return moving_std


def detect_motion_start(data, window_duration=1.0, threshold_multiplier=2.0):
    """Detect when significant motion starts using moving standard deviation."""
    # Use gyroscope magnitude as it's most sensitive to motion
    gyro_mag = np.sqrt(data['gyro_x']**2 + data['gyro_y']**2 + data['gyro_z']**2)
    gyro_moving_std = calculate_moving_std(gyro_mag, data['timestamps'], window_duration)
    
    # Calculate baseline from first 10% of data
    baseline_samples = int(len(gyro_moving_std) * 0.1)
    baseline_std = np.mean(gyro_moving_std[:baseline_samples])
    threshold = baseline_std * threshold_multiplier
    
    # Find first point where moving std exceeds threshold
    motion_indices = np.where(gyro_moving_std > threshold)[0]
    
    if len(motion_indices) > 0:
        motion_start_idx = motion_indices[0]
        motion_start_time = data['timestamps'][motion_start_idx] - data['timestamps'][0]
        return motion_start_idx, motion_start_time, baseline_std, threshold
    else:
        return None, None, baseline_std, threshold


def plot_imu_data(data, output_path: str = "imu_analysis.png"):
    """Plot IMU data and moving standard deviation analysis."""
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))

    # Normalize timestamps to start at 0
    t = data['timestamps'] - data['timestamps'][0]

    # Plot raw accelerometer data
    ax = axes[0, 0]
    ax.plot(t, data['accel_x'], label='X', alpha=0.7)
    ax.plot(t, data['accel_y'], label='Y', alpha=0.7)
    ax.plot(t, data['accel_z'], label='Z', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (m/s²)')
    ax.set_title('Raw Accelerometer Data')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot raw gyroscope data
    ax = axes[0, 1]
    ax.plot(t, data['gyro_x'], label='X', alpha=0.7)
    ax.plot(t, data['gyro_y'], label='Y', alpha=0.7)
    ax.plot(t, data['gyro_z'], label='Z', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angular Velocity (rad/s)')
    ax.set_title('Raw Gyroscope Data')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot accelerometer and gyroscope magnitudes
    ax = axes[1, 0]
    accel_mag = np.sqrt(data['accel_x']**2 + data['accel_y']**2 + data['accel_z']**2)
    ax.plot(t, accel_mag, 'b-', alpha=0.7, label='Magnitude')
    ax.plot(t, np.abs(data['accel_x']), 'r-', alpha=0.5, label='|X|')
    ax.plot(t, np.abs(data['accel_y']), 'g-', alpha=0.5, label='|Y|')
    ax.plot(t, np.abs(data['accel_z']), 'orange', alpha=0.5, label='|Z|')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration Magnitude (m/s²)')
    ax.set_title('Accelerometer Magnitude')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    gyro_mag = np.sqrt(data['gyro_x']**2 + data['gyro_y']**2 + data['gyro_z']**2)
    ax.plot(t, gyro_mag, 'b-', alpha=0.7, label='Magnitude')
    ax.plot(t, np.abs(data['gyro_x']), 'r-', alpha=0.5, label='|X|')
    ax.plot(t, np.abs(data['gyro_y']), 'g-', alpha=0.5, label='|Y|')
    ax.plot(t, np.abs(data['gyro_z']), 'orange', alpha=0.5, label='|Z|')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angular Velocity Magnitude (rad/s)')
    ax.set_title('Gyroscope Magnitude')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Moving standard deviation plots (1 second window)
    ax = axes[2, 0]
    accel_mag = np.sqrt(data['accel_x']**2 + data['accel_y']**2 + data['accel_z']**2)
    accel_moving_std = calculate_moving_std(accel_mag, data['timestamps'], window_duration=1.0)
    ax.plot(t, accel_moving_std, 'b-', alpha=0.7, label='Accel Magnitude')
    
    # Add individual axis moving std
    accel_x_std = calculate_moving_std(data['accel_x'], data['timestamps'], window_duration=1.0)
    accel_y_std = calculate_moving_std(data['accel_y'], data['timestamps'], window_duration=1.0)
    accel_z_std = calculate_moving_std(data['accel_z'], data['timestamps'], window_duration=1.0)
    ax.plot(t, accel_x_std, 'r-', alpha=0.5, label='X')
    ax.plot(t, accel_y_std, 'g-', alpha=0.5, label='Y')
    ax.plot(t, accel_z_std, 'orange', alpha=0.5, label='Z')
    
    # Add threshold line for motion detection
    baseline_std = np.mean(accel_moving_std[:int(len(accel_moving_std)*0.1)])  # First 10%
    threshold = baseline_std * 2.0
    ax.axhline(threshold, color='red', linestyle='--', alpha=0.8, label=f'2x Baseline: {threshold:.3f}')
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Moving Std (1s window) (m/s²)')
    ax.set_title('Accelerometer Moving Standard Deviation')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[2, 1]
    gyro_mag = np.sqrt(data['gyro_x']**2 + data['gyro_y']**2 + data['gyro_z']**2)
    gyro_moving_std = calculate_moving_std(gyro_mag, data['timestamps'], window_duration=1.0)
    ax.plot(t, gyro_moving_std, 'b-', alpha=0.7, label='Gyro Magnitude')
    
    # Add individual axis moving std
    gyro_x_std = calculate_moving_std(data['gyro_x'], data['timestamps'], window_duration=1.0)
    gyro_y_std = calculate_moving_std(data['gyro_y'], data['timestamps'], window_duration=1.0)
    gyro_z_std = calculate_moving_std(data['gyro_z'], data['timestamps'], window_duration=1.0)
    ax.plot(t, gyro_x_std, 'r-', alpha=0.5, label='X')
    ax.plot(t, gyro_y_std, 'g-', alpha=0.5, label='Y')
    ax.plot(t, gyro_z_std, 'orange', alpha=0.5, label='Z')
    
    # Add threshold line for motion detection
    baseline_std = np.mean(gyro_moving_std[:int(len(gyro_moving_std)*0.1)])  # First 10%
    threshold = baseline_std * 2.0
    ax.axhline(threshold, color='red', linestyle='--', alpha=0.8, label=f'2x Baseline: {threshold:.3f}')
    
    # Detect motion start
    motion_indices = np.where(gyro_moving_std > threshold)[0]
    if len(motion_indices) > 0:
        motion_start_time = t[motion_indices[0]]
        ax.axvline(motion_start_time, color='red', linestyle='-', alpha=0.8, label=f'Motion Start: {motion_start_time:.1f}s')
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Moving Std (1s window) (rad/s)')
    ax.set_title('Gyroscope Moving Standard Deviation')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"Plot saved to {output_path}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(description='Analyze IMU data from rosbag')
    parser.add_argument('bag_path', type=str, help='Path to rosbag')
    parser.add_argument('--topic', type=str, default='/ouster/imu', help='IMU topic name')
    parser.add_argument('--output', type=str, default='imu_analysis.png', help='Output plot path')
    parser.add_argument('--duration', type=float, help='Duration in seconds to analyze for statistics (default: analyze all data)')
    args = parser.parse_args()

    print(f"Reading IMU data from {args.bag_path} topic {args.topic}...")
    data = read_imu_from_bag(args.bag_path, args.topic)

    if data is None:
        return

    total_duration = data['timestamps'][-1] - data['timestamps'][0]
    print(f"Read {len(data['timestamps'])} IMU messages")
    print(f"Total duration: {total_duration:.2f} seconds")
    print(f"Rate: {len(data['timestamps']) / total_duration:.1f} Hz")

    # Validate duration argument
    if args.duration is not None:
        if args.duration <= 0:
            print("Error: Duration must be positive")
            return
        if args.duration > total_duration:
            print(f"Warning: Requested duration ({args.duration}s) is longer than available data ({total_duration:.2f}s)")
            print("Using all available data for analysis")
            args.duration = None

    # Motion detection analysis
    print("\n--- Motion Detection Analysis ---")
    motion_idx, motion_time, baseline_std, threshold = detect_motion_start(data)
    
    if motion_time is not None:
        print(f"Motion start detected at: {motion_time:.2f} seconds")
        print(f"Baseline gyro std: {baseline_std:.6f} rad/s")
        print(f"Detection threshold (2x baseline): {threshold:.6f} rad/s")
        print(f"Inactive period: {motion_time:.2f} seconds ({motion_idx} samples)")
        print(f"Active period: {total_duration - motion_time:.2f} seconds")
        
        # Suggest using detected motion start for analysis if no duration specified
        if args.duration is None:
            print(f"\nSuggestion: Use --duration {motion_time:.1f} to analyze only the inactive period")
            print(f"           or start analysis from {motion_time:.1f}s to focus on active motion")
    else:
        print("No clear motion start detected (data appears consistently active)")

    print("\n--- Moving Standard Deviation Analysis ---")
    results, analysis_data = analyze_moving_std_stats(data, args.duration)

    print("\nAccelerometer Moving Std (m/s²):")
    for axis in ['accel_x', 'accel_y', 'accel_z']:
        r = results[axis]
        print(f"  {axis}: mean={r['mean']:.6f}, std={r['std']:.6f}, P95={r['p95']:.6f}, P99={r['p99']:.6f}, max={r['max']:.6f}")

    print("\nGyroscope Moving Std (rad/s):")
    for axis in ['gyro_x', 'gyro_y', 'gyro_z']:
        r = results[axis]
        print(f"  {axis}: mean={r['mean']:.6f}, std={r['std']:.6f}, P95={r['p95']:.6f}, P99={r['p99']:.6f}, max={r['max']:.6f}")

    # Use analysis_data for thresholds if duration was specified, otherwise use full data
    threshold_data = analysis_data if analysis_data is not None else data
    
    # Calculate moving std thresholds based on analyzed data
    accel_mag = np.sqrt(threshold_data['accel_x']**2 + threshold_data['accel_y']**2 + threshold_data['accel_z']**2)
    gyro_mag = np.sqrt(threshold_data['gyro_x']**2 + threshold_data['gyro_y']**2 + threshold_data['gyro_z']**2)
    
    accel_moving_std = calculate_moving_std(accel_mag, threshold_data['timestamps'], window_duration=1.0)
    gyro_moving_std = calculate_moving_std(gyro_mag, threshold_data['timestamps'], window_duration=1.0)

    print("\n--- Suggested Moving Std Thresholds (based on analyzed data) ---")
    print(f"Accelerometer magnitude moving std:")
    print(f"  95th percentile: {np.percentile(accel_moving_std, 95):.6f} m/s²")
    print(f"  99th percentile: {np.percentile(accel_moving_std, 99):.6f} m/s²")
    print(f"  Maximum: {np.max(accel_moving_std):.6f} m/s²")
    
    print(f"Gyroscope magnitude moving std:")
    print(f"  95th percentile: {np.percentile(gyro_moving_std, 95):.6f} rad/s")
    print(f"  99th percentile: {np.percentile(gyro_moving_std, 99):.6f} rad/s")
    print(f"  Maximum: {np.max(gyro_moving_std):.6f} rad/s")

    # Generate plot using ALL data (not limited by duration)
    print(f"\nGenerating plot of all {total_duration:.2f} seconds of data...")
    plot_imu_data(data, args.output)


if __name__ == '__main__':
    main()
