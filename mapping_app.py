import os
import json
import subprocess
import shutil
from pathlib import Path

# Default mapping settings with options/range format
DEFAULT_MAPPING_SETTINGS = {
    "preview_voxel_size": {
        "options": [5, 10, 15],  # cm
        "current_selection": 1
    },
    "file_format": {
        "options": ["PLY", "LAS", "LAZ"],
        "current_selection": 1
    },
    "map_quality": {
        "options": ["Low", "Mid", "High"],
        "current_selection": 1
    }
}


class MappingApp:
    """Handles mapping/spatial map settings and Docker control."""

    def __init__(self, data_path: str, catalog=None):
        self.data_path = data_path
        self.settings_path = os.path.join(data_path, "mapping_settings.json")
        self.settings = self._load_settings()
        self.catalog = catalog

        # Docker and mapping configuration
        self.docker_image = "osrf/ros:humble-desktop-full-jammy"
        self.artifact_dir = "/tmp/mapping_artifact"
        self.container_name = "fastlio_mapping"
        self.is_running = False

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
        return self.settings

    def save_settings(self, new_settings: dict):
        """Update and save settings."""
        self.settings = new_settings
        self._save_settings()
        return self.settings

    def _get_setting_value(self, key: str):
        """Get the actual value of a setting (resolves current_selection)."""
        setting = self.settings.get(key, {})
        if "options" in setting:
            idx = setting.get("current_selection", 0)
            return setting["options"][idx]
        elif "range" in setting:
            return setting.get("current_selection", setting["range"][0])
        return None

    def start(self):
        """Start the mapping process by launching Docker container."""
        if self.is_running:
            raise RuntimeError("Mapping is already running")

        # Create artifact directory
        os.makedirs(self.artifact_dir, exist_ok=True)

        # Get file format from settings
        file_format = self._get_setting_value("file_format")
        if file_format:
            file_format = file_format.lower()  # Convert PLY -> ply
        else:
            file_format = "ply"

        # Stop any existing container with this name
        subprocess.run(["docker", "stop", self.container_name],
                      capture_output=True, check=False)
        subprocess.run(["docker", "rm", self.container_name],
                      capture_output=True, check=False)

        # Launch Docker container with Fast-LIO
        docker_cmd = [
            "docker", "run",
            "--name", self.container_name,
            "-v", "/home/linh/ros2_ws:/ros2_ws",
            "-v", "/f/shared_data:/shared_data",
            "-v", f"{self.artifact_dir}:/tmp/mapping_artifact",
            "-d",  # Detached mode
            self.docker_image,
            "bash", "-c",
            f"source /opt/ros/humble/setup.bash && "
            f"source /ros2_ws/install/setup.bash && "
            f"ros2 launch fast_lio mapping_with_bridge.launch.py "
            f"artifact_dir:=/tmp/mapping_artifact file_format:={file_format}"
        ]

        try:
            result = subprocess.run(docker_cmd, capture_output=True, text=True, check=True)
            self.is_running = True
            return {"status": "started", "container_id": result.stdout.strip()}
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Failed to start mapping container: {e.stderr}")

    def get_init_status(self):
        """Get initialization status by reading the status file."""
        status_file = os.path.join(self.artifact_dir, "initStatus")

        if not os.path.exists(status_file):
            return {"status": "INITIALIZING"}

        try:
            with open(status_file, 'r') as f:
                status = f.read().strip()
            return {"status": status}
        except Exception as e:
            return {"status": "INITIALIZING", "error": str(e)}

    def stop(self):
        """Stop the mapping process and finalize results."""
        if not self.is_running:
            raise RuntimeError("Mapping is not running")

        # Stop the Docker container
        try:
            subprocess.run(["docker", "stop", self.container_name],
                          capture_output=True, text=True, check=True, timeout=30)
        except subprocess.TimeoutExpired:
            # Force kill if graceful shutdown times out
            subprocess.run(["docker", "kill", self.container_name],
                          capture_output=True, check=False)

        # Run finalization script
        file_format = self._get_setting_value("file_format")
        if file_format:
            file_format = file_format.lower()
        else:
            file_format = "ply"

        finalize_script = "/home/linh/ros2_ws/src/FAST_LIO/scripts/finalize_mapping.py"
        try:
            subprocess.run(
                ["python3", finalize_script, self.artifact_dir, "--file-format", file_format],
                capture_output=True, text=True, check=True, timeout=300
            )
        except subprocess.CalledProcessError as e:
            print(f"Warning: Finalization script failed: {e.stderr}")
        except subprocess.TimeoutExpired:
            print("Warning: Finalization script timed out")

        # Add to catalog if catalog is available
        if self.catalog:
            self._add_to_catalog(file_format)

        # Cleanup
        subprocess.run(["docker", "rm", self.container_name],
                      capture_output=True, check=False)
        self.is_running = False

        return {"status": "stopped"}

    def _add_to_catalog(self, file_format: str):
        """Add mapping result to catalog and move artifacts."""
        import uuid
        # File paths in artifact directory
        map_file = os.path.join(self.artifact_dir, f"map_result.{file_format}")
        thumbnail_file = os.path.join(self.artifact_dir, "thumbnail.png")
        metadata_file = os.path.join(self.artifact_dir, "map_metadata.json")
        sensor_raw_dir = os.path.join(self.artifact_dir, "sensor_raw")

        # Build contents dict for catalog
        contents = {}

        if os.path.exists(map_file):
            contents["pointcloud"] = map_file

        if os.path.exists(thumbnail_file):
            contents["thumbnail"] = thumbnail_file

        if os.path.exists(metadata_file):
            contents["metadata"] = metadata_file

        if os.path.exists(sensor_raw_dir):
            # Get all files in sensor_raw directory
            sensor_files = [os.path.join(sensor_raw_dir, f)
                          for f in os.listdir(sensor_raw_dir)
                          if os.path.isfile(os.path.join(sensor_raw_dir, f))]
            if sensor_files:
                contents["raw"] = sensor_files

        # Add to catalog (catalog will move files and create entry)
        item_uuid = self.catalog.add_item("Scans", contents)

        return item_uuid
