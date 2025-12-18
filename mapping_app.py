import os
import json
import threading
import time
from pathlib import Path
from enum import Enum
from typing import Optional
import asyncio

from ros2 import StartEverything, StopEverything, GetInitStatus
from finalize_mapping import do_finalize

# Default mapping settings with options/range format
DEFAULT_MAPPING_SETTINGS = {
    "preview_voxel_size": {
        "options": [5, 10, 15],  # cm
        "current_selection": 1
    },
    "file_format": {
        "options": ["PLY", "PCD", "LAS", "LAZ"],
        "current_selection": 1
    },
    "map_quality": {
        "options": ["Low", "Mid", "High"],
        "current_selection": 1
    }
}


class MappingState(Enum):
    """
    Mapping operation state machine.

    Lifecycle:
    IDLE → STARTING → INITIALIZING → RUNNING → STOPPING → IDLE
    
    ERROR state can be entered from any state on failure.
    
    """
    IDLE = "idle"
    STARTING = "starting"              # Creating ROS2 nodes
    INITIALIZING = "initializing"      # Nodes exist, waiting for IMU stabilization
    RUNNING = "running"                # IMU stabilized, actively mapping
    STOPPING = "stopping"              # Destroying nodes
    ERROR = "error"


class MappingApp:
    """Handles mapping/spatial scan settings and ROS2 node lifecycle management."""

    def __init__(self, data_path: str, catalog=None):
        self.settings_path = os.path.join(data_path, "mapping_settings.json")
        self.settings = self._load_settings()
        self.active_settings = self.settings.copy()
        self.catalog = catalog

        # ROS2 and mapping configuration
        self.artifact_dir = os.path.join(data_path, "mapping_artifact")

        # State machine for start/stop operations
        self.state = MappingState.IDLE
        self._state_lock = asyncio.Lock()
        self._process_error = None
        
        self._polling_task = None


    def _load_settings(self):
        """Load settings from disk or return defaults."""
        try:
            with open(self.settings_path, 'r') as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return DEFAULT_MAPPING_SETTINGS.copy()

    def _save_settings(self):
        """Save settings to disk."""
        with open(self.settings_path, 'w') as f:
            json.dump(self.settings, f, indent=2)

    def get_settings(self):
        """Return current settings."""
        return self.active_settings

    def _validate_settings(self, settings: dict) -> tuple[dict, list]:
        """Validate and sanitize settings.

        Handles two formats:
        1. Full format: {"key": {"options": [...], "current_selection": N}}
        2. Flat format: {"key": value} - value matched against current options

        Returns:
            tuple: (validated_settings, list of invalid keys)
        """
        validated = {}
        invalid_keys = []

        for key, value in settings.items():
            # Check if this is a known setting
            if key not in self.settings:
                invalid_keys.append(key)
                continue

            current_setting = self.settings[key]

            if isinstance(value, dict):
                # Full format: {"options": [...], "current_selection": N}
                if "options" in value and "current_selection" in value:
                    options = value["options"]
                    selection = value["current_selection"]
                    if not isinstance(selection, int) or selection < 0 or selection >= len(options):
                        selection = 0  # Reset to default if invalid
                    validated[key] = {
                        "options": options,
                        "current_selection": selection
                    }
                elif "range" in value and "current_selection" in value:
                    range_vals = value["range"]
                    selection = value["current_selection"]
                    if not isinstance(selection, (int, float)):
                        selection = range_vals[0]
                    selection = max(range_vals[0], min(range_vals[1], selection))
                    validated[key] = {
                        "range": range_vals,
                        "current_selection": selection
                    }
                else:
                    validated[key] = value
            else:
                # Flat format: value to match against options
                if "options" in current_setting:
                    options = current_setting["options"]
                    if value in options:
                        # Valid value - find its index
                        validated[key] = {
                            "options": options,
                            "current_selection": options.index(value)
                        }
                    else:
                        # Invalid value - mark as invalid, keep current setting
                        invalid_keys.append(key)
                        validated[key] = current_setting.copy()
                elif "range" in current_setting:
                    range_vals = current_setting["range"]
                    if isinstance(value, (int, float)) and range_vals[0] <= value <= range_vals[1]:
                        validated[key] = {
                            "range": range_vals,
                            "current_selection": value
                        }
                    else:
                        invalid_keys.append(key)
                        validated[key] = current_setting.copy()
                else:
                    validated[key] = value

        # Preserve settings not in the update
        for key in self.settings:
            if key not in validated:
                validated[key] = self.settings[key].copy() if isinstance(self.settings[key], dict) else self.settings[key]

        return validated, invalid_keys

    def save_settings(self, new_settings: dict):
        """Update and save settings.

        When idle, active_settings is also updated immediately.
        When mapping is running, active_settings updates after stop.

        Raises:
            ValueError: If any settings have invalid values (still updates valid ones)
        """
        validated, invalid_keys = self._validate_settings(new_settings)
        self.settings = validated
        self._save_settings()

        # When idle, sync active_settings immediately
        if self.state == MappingState.IDLE:
            self.active_settings = self.settings.copy()

        # Raise error if there were invalid values
        if invalid_keys:
            raise ValueError(f"Invalid settings: {', '.join(invalid_keys)}")

        return self.settings

    def _get_setting_value(self, key: str):
        """Get the actual value of a setting (resolves current_selection)."""
        setting = self.active_settings.get(key, {})
        if "options" in setting:
            idx = setting.get("current_selection", 0)
            return setting["options"][idx]
        elif "range" in setting:
            return setting.get("current_selection", setting["range"][0])
        return None

    async def _poll_init_status(self):
        """Poll IMU initialization status and update state machine."""
        while True:
            imu_status = GetInitStatus()
            async with self._state_lock:
                if imu_status == "TRACKING" and self.state == MappingState.STARTING:
                    self.state = MappingState.INITIALIZING
                elif imu_status == "STABILIZED" and self.state == MappingState.INITIALIZING:
                    self.state = MappingState.RUNNING
                    break
            await asyncio.sleep(1.0)

    async def _do_start(self):
        os.makedirs(self.artifact_dir, exist_ok=True)
        file_format = self._get_setting_value("file_format")

        await StartEverything(file_format, self.artifact_dir)
        self._polling_task = asyncio.create_task(self._poll_init_status())

    async def start(self):
        """
        Start the mapping process by launching ROS2 Fast-LIO nodes.

        State transition: IDLE → STARTING → INITIALIZING → RUNNING

        Returns:
            dict: Status message

        Raises:
            RuntimeError: If mapping is in an invalid state for starting
        """
        async with self._state_lock:
            # Idempotent: already running or initializing
            if self.state in [MappingState.RUNNING, MappingState.INITIALIZING]:
                return {"details": "already_running", "state": self.state.value}

            # Reject if in transition
            if self.state in [MappingState.STARTING, MappingState.STOPPING]:
                raise RuntimeError(f"Cannot start: mapping is {self.state.value}")

            self.state = MappingState.STARTING

        self.active_settings = self.settings.copy()
        await self._do_start()
            
        return {"details": "started", "state": self.state.value}

    def get_init_status(self):
        """
        Get the IMU initialization status.

        Returns:
            dict: imu status
        """
        return {
            "imu_status": GetInitStatus()
        }

    async def _do_stop(self):
        await StopEverything()

        # Run finalization script
        file_format = self._get_setting_value("file_format").lower()

        await asyncio.to_thread(do_finalize, self.artifact_dir, file_format)

        if self.catalog:
            self.catalog.add_item("Scan", self.artifact_dir)

        async with self._state_lock:
            self.state = MappingState.IDLE

    async def stop(self):
        """
        Stop the mapping process and finalize results.

        Returns:
            dict: Status message

        Raises:
            RuntimeError: If mapping is in an invalid state for stopping
        """
        async with self._state_lock:
            # Idempotent: already idle
            if self.state == MappingState.IDLE:
                return {"details": "already_stopped", "state": self.state.value}

            # Reject if in transition
            if self.state in [MappingState.STARTING, MappingState.INITIALIZING, MappingState.STOPPING]:
                raise RuntimeError(f"Cannot stop: mapping is {self.state.value}")

            self.state = MappingState.STOPPING

        await self._do_stop()
        self.active_settings = self.settings.copy()
        return {"details": "stopped", "state": self.state.value}

    def get_state(self):
        """
        Get current mapping state for status monitoring.

        Returns:
            dict: Current state
        """
        return {
            "state": self.state.value,
        }
