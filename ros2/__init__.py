"""
ROS2 wrapper for managing the mapping system lifecycle.

Provides functions to start/stop all ROS2 nodes and check initialization status.
"""

import os
import asyncio
import threading
from pathlib import Path
from typing import Optional

from launch import LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


# Global launch service instance
_launch_service: Optional[LaunchService] = None
_launch_thread: Optional[threading.Thread] = None
_artifact_dir: str = ""


def _run_launch_service(launch_service: LaunchService):
    """Run the launch service in a separate thread."""
    launch_service.run()


async def StartEverything(file_format: str, artifact_dir: str):
    """
    Create a launch service and start all nodes using the top-level launch description.
    Start asynchronously, returning immediately.

    Args:
        file_format: Output file format (PLY, PCD, LAS, LAZ)
        artifact_dir: Directory to store all mapping artifacts
    """
    global _launch_service, _launch_thread, _artifact_dir

    # Store artifact_dir for GetInitStatus
    _artifact_dir = artifact_dir

    # Ensure artifact directory exists
    os.makedirs(artifact_dir, exist_ok=True)

    # Get the path to the launch file
    current_dir = Path(__file__).parent.absolute()
    launch_file_path = str(current_dir / 'toplevel.launch.py')

    # Create launch service
    _launch_service = LaunchService()

    # Create launch description with arguments
    # Note: For now, we default to development_mode=true
    # You may want to add a parameter or setting to control this
    launch_description_source = PythonLaunchDescriptionSource(launch_file_path)

    include_launch = IncludeLaunchDescription(
        launch_description_source,
        launch_arguments={
            'file_format': file_format,
            'artifact_dir': artifact_dir,
            'development_mode': 'true'  # TODO: Make this configurable via settings
        }.items()
    )

    # Include the launch description
    _launch_service.include_launch_description(
        include_launch.visit(None)
    )

    # Start the launch service in a background thread
    _launch_thread = threading.Thread(
        target=_run_launch_service,
        args=(_launch_service,),
        daemon=True
    )
    _launch_thread.start()

    # Give it a moment to start
    await asyncio.sleep(0.5)


async def StopEverything():
    """
    Shutdown the launch service and all running nodes.
    """
    global _launch_service, _launch_thread

    if _launch_service is not None:
        # Request shutdown
        await asyncio.to_thread(_launch_service.shutdown)

        # Wait for the thread to finish (with timeout)
        if _launch_thread is not None and _launch_thread.is_alive():
            _launch_thread.join(timeout=5.0)

        # Reset global state
        _launch_service = None
        _launch_thread = None


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
