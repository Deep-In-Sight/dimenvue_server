"""
ROS2 wrapper for managing the mapping system lifecycle.

Provides functions to start/stop all ROS2 nodes and check initialization status.
"""

import os
import asyncio
import subprocess
import signal
from pathlib import Path
from typing import Optional


# Global process instance
_launch_process: Optional[subprocess.Popen] = None
_artifact_dir: str = ""


async def StartEverything(file_format: str, artifact_dir: str, bag_path: str = "/shared_data/test_bag"):
    """
    Start all ROS2 nodes using ros2 launch subprocess.

    Args:
        file_format: Output file format (PLY, PCD, LAS, LAZ)
        artifact_dir: Directory to store all mapping artifacts
        bag_path: Path to the rosbag for development mode playback
    """
    global _launch_process, _artifact_dir

    # Store artifact_dir for GetInitStatus
    _artifact_dir = artifact_dir

    # Ensure artifact directory exists
    os.makedirs(artifact_dir, exist_ok=True)

    # Get the path to the launch file
    current_dir = Path(__file__).parent.absolute()
    launch_file_path = str(current_dir / 'toplevel.launch.py')

    # Build the ros2 launch command
    cmd = [
        'ros2', 'launch',
        launch_file_path,
        f'file_format:={file_format}',
        f'artifact_dir:={artifact_dir}',
        f'bag_path:={bag_path}',
        'development_mode:=false'  # TODO: Make this configurable via settings
    ]

    # Start the launch process
    # Don't capture stdout/stderr - let it flow to console to avoid blocking
    _launch_process = subprocess.Popen(
        cmd,
        stdout=None,  # Inherit from parent (shows in console)
        stderr=None,  # Inherit from parent
        preexec_fn=os.setsid  # Create new process group for clean shutdown
    )

    # Give it a moment to start
    await asyncio.sleep(0.5)


async def StopEverything():
    """
    Shutdown all running ROS2 nodes.
    """
    global _launch_process

    if _launch_process is not None:
        try:
            # Send SIGINT to the process group (like Ctrl+C)
            os.killpg(os.getpgid(_launch_process.pid), signal.SIGINT)

            # Wait for graceful shutdown with timeout
            try:
                await asyncio.wait_for(
                    asyncio.to_thread(_launch_process.wait),
                    timeout=10.0
                )
            except asyncio.TimeoutError:
                # Force kill if graceful shutdown fails
                os.killpg(os.getpgid(_launch_process.pid), signal.SIGKILL)
                await asyncio.to_thread(_launch_process.wait)

        except ProcessLookupError:
            # Process already terminated
            pass
        finally:
            _launch_process = None


def GetInitStatus() -> str:
    """
    Return the IMU stabilization status.
    Read from the imu monitor result file.

    Returns:
        Status string: "TRACKING", "STABILIZED", or "UNKNOWN" if file doesn't exist
    """
    global _artifact_dir

    if not _artifact_dir:
        return "UNKNOWN"

    status_file = os.path.join(_artifact_dir, "imu_stabilization_status.txt")

    try:
        with open(status_file, 'r') as f:
            status = f.read().strip()
            return status
    except FileNotFoundError:
        return "UNKNOWN"
    except Exception as e:
        print(f"Error reading IMU status file: {e}")
        return "UNKNOWN"
