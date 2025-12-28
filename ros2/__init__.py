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

# Persistent state subscriber
_state_subscriber: Optional[subprocess.Popen] = None
_state_reader_task: Optional[asyncio.Task] = None
_current_state: str = "UNKNOWN"


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

    # Start persistent state subscriber
    await _start_state_subscriber()


async def StopEverything():
    """
    Shutdown all running ROS2 nodes.
    """
    global _launch_process

    # Stop state subscriber first
    await _stop_state_subscriber()

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


async def _read_state_loop():
    """Background task to read state updates from the subscriber process."""
    global _current_state, _state_subscriber

    while _state_subscriber and _state_subscriber.poll() is None:
        try:
            # Read line asynchronously
            line = await asyncio.to_thread(_state_subscriber.stdout.readline)
            if not line:
                break
            line = line.strip()
            # Skip separator lines
            if line and line != "---":
                if line in ("IDLE", "STABILIZING", "RUNNING"):
                    _current_state = line
        except Exception as e:
            print(f"Error reading state: {e}")
            break


async def _start_state_subscriber():
    """Start the persistent state subscriber process."""
    global _state_subscriber, _state_reader_task, _current_state

    _current_state = "UNKNOWN"

    # Start ros2 topic echo without --once to get continuous updates
    _state_subscriber = subprocess.Popen(
        ['ros2', 'topic', 'echo', '--field', 'data', '/mappingState'],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        bufsize=1  # Line buffered
    )

    # Start background reader task
    _state_reader_task = asyncio.create_task(_read_state_loop())


async def _stop_state_subscriber():
    """Stop the persistent state subscriber process."""
    global _state_subscriber, _state_reader_task, _current_state

    if _state_reader_task:
        _state_reader_task.cancel()
        try:
            await _state_reader_task
        except asyncio.CancelledError:
            pass
        _state_reader_task = None

    if _state_subscriber:
        _state_subscriber.terminate()
        try:
            _state_subscriber.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            _state_subscriber.kill()
        _state_subscriber = None

    _current_state = "UNKNOWN"


def GetInitStatus() -> str:
    """
    Return the cached mapping state from /mappingState topic.

    Returns:
        Status string: "IDLE", "STABILIZING", "RUNNING", or "UNKNOWN" if unavailable
    """
    return _current_state
