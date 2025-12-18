"""
Unit tests for finalize_mapping.py

Test IDs: UNIT-FIN-001 to UNIT-FIN-008
"""

import json
import os
import sys
import tempfile
import shutil
import pytest
import numpy as np

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from finalize_mapping import do_finalize, load_pointcloud, generate_thumbnail, calculate_area_volume


def create_mock_pcd_file(filepath: str, num_points: int = 1000):
    """
    Create a mock PCD file with random point cloud data.
    Uses ASCII format for simplicity.
    """
    # Generate random points in a reasonable range
    np.random.seed(42)  # For reproducibility
    points = np.random.randn(num_points, 3) * 10  # Points in ~[-30, 30] range

    # PCD ASCII format
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {num_points}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {num_points}
DATA ascii
"""

    with open(filepath, 'w') as f:
        f.write(header)
        for point in points:
            f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")

    return points


def create_mock_ply_file(filepath: str, num_points: int = 1000):
    """Create a mock PLY file with random point cloud data."""
    np.random.seed(42)
    points = np.random.randn(num_points, 3) * 10

    header = f"""ply
format ascii 1.0
element vertex {num_points}
property float x
property float y
property float z
end_header
"""

    with open(filepath, 'w') as f:
        f.write(header)
        for point in points:
            f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")

    return points


def create_mock_sensor_raw(artifact_dir: str):
    """Create mock sensor_raw directory with rosbag metadata."""
    sensor_raw_dir = os.path.join(artifact_dir, "sensor_raw")
    os.makedirs(sensor_raw_dir, exist_ok=True)

    # Create mock metadata.yaml
    metadata = {
        "rosbag2_bagfile_information": {
            "version": 4,
            "storage_identifier": "sqlite3",
            "duration": {"nanoseconds": 30000000000},
            "starting_time": {"nanoseconds_since_epoch": 1700000000000000000},
            "message_count": 3000,
            "topics_with_message_count": [
                {"topic_metadata": {"name": "/ouster/points"}, "message_count": 300},
                {"topic_metadata": {"name": "/ouster/imu"}, "message_count": 2700}
            ]
        }
    }

    metadata_path = os.path.join(sensor_raw_dir, "metadata.yaml")
    import yaml
    with open(metadata_path, 'w') as f:
        yaml.dump(metadata, f)


class TestDoFinalizeValidPCD:
    """UNIT-FIN-001: Test do_finalize with valid PCD file"""

    @pytest.fixture
    def artifact_dir(self):
        """Create temp artifact directory with mock data."""
        tmpdir = tempfile.mkdtemp(prefix="test_finalize_")

        # Create mock PCD file
        pcd_path = os.path.join(tmpdir, "map_result.PCD")
        create_mock_pcd_file(pcd_path, num_points=1000)

        # Create mock sensor_raw
        create_mock_sensor_raw(tmpdir)

        yield tmpdir

        # Cleanup
        shutil.rmtree(tmpdir, ignore_errors=True)

    def test_do_finalize_valid_pcd(self, artifact_dir):
        """
        UNIT-FIN-001: do_finalize with valid PCD file should:
        - Return without error
        - Generate thumbnail.png (may fail in headless mode)
        - Generate map_metadata.json with valid structure
        """
        # Run finalize
        do_finalize(artifact_dir, "PCD")

        # Check thumbnail - may not be generated in headless/CI environment
        thumbnail_path = os.path.join(artifact_dir, "thumbnail.png")
        if os.path.exists(thumbnail_path):
            assert os.path.getsize(thumbnail_path) > 0, "thumbnail.png should not be empty"
        else:
            # Thumbnail generation can fail in headless mode - this is acceptable
            print("Warning: thumbnail.png not generated (headless mode)")

        # Check metadata generated - this should always work
        metadata_path = os.path.join(artifact_dir, "map_metadata.json")
        assert os.path.exists(metadata_path), "map_metadata.json should be generated"

        with open(metadata_path, 'r') as f:
            metadata = json.load(f)

        # Verify metadata structure
        assert "num_points" in metadata, "metadata should contain num_points"
        assert metadata["num_points"] == 1000, "num_points should match input"

        assert "area" in metadata, "metadata should contain area"
        assert metadata["area"] > 0, "area should be positive"

        assert "volume" in metadata, "metadata should contain volume"
        assert metadata["volume"] > 0, "volume should be positive"

        assert "data_size" in metadata, "metadata should contain data_size"


class TestDoFinalizeMissingMap:
    """UNIT-FIN-002: Test do_finalize with missing map file"""

    @pytest.fixture
    def empty_artifact_dir(self):
        """Create empty temp artifact directory."""
        tmpdir = tempfile.mkdtemp(prefix="test_finalize_empty_")
        yield tmpdir
        shutil.rmtree(tmpdir, ignore_errors=True)

    def test_do_finalize_missing_map(self, empty_artifact_dir):
        """
        UNIT-FIN-002: do_finalize with missing map file should:
        - Raise SystemExit (current behavior calls sys.exit(1))

        Note: This tests current behavior. Ideally should raise FileNotFoundError.
        """
        with pytest.raises(SystemExit) as exc_info:
            do_finalize(empty_artifact_dir, "PCD")

        assert exc_info.value.code == 1, "Should exit with code 1"


class TestDoFinalizeFormats:
    """UNIT-FIN-003: Test do_finalize with various formats"""

    @pytest.fixture
    def artifact_dir_ply(self):
        """Create temp artifact directory with PLY file."""
        tmpdir = tempfile.mkdtemp(prefix="test_finalize_ply_")
        ply_path = os.path.join(tmpdir, "map_result.PLY")
        create_mock_ply_file(ply_path, num_points=500)
        yield tmpdir
        shutil.rmtree(tmpdir, ignore_errors=True)

    @pytest.fixture
    def artifact_dir_pcd(self):
        """Create temp artifact directory with PCD file."""
        tmpdir = tempfile.mkdtemp(prefix="test_finalize_pcd_")
        pcd_path = os.path.join(tmpdir, "map_result.PCD")
        create_mock_pcd_file(pcd_path, num_points=500)
        yield tmpdir
        shutil.rmtree(tmpdir, ignore_errors=True)

    def test_do_finalize_ply_format(self, artifact_dir_ply):
        """Test do_finalize with PLY format."""
        do_finalize(artifact_dir_ply, "PLY")

        # Thumbnail may fail in headless mode
        thumbnail_path = os.path.join(artifact_dir_ply, "thumbnail.png")
        if not os.path.exists(thumbnail_path):
            print("Warning: thumbnail.png not generated (headless mode)")

        # Metadata should always be generated
        assert os.path.exists(os.path.join(artifact_dir_ply, "map_metadata.json"))

        with open(os.path.join(artifact_dir_ply, "map_metadata.json")) as f:
            metadata = json.load(f)
        assert metadata["num_points"] == 500

    def test_do_finalize_pcd_format(self, artifact_dir_pcd):
        """Test do_finalize with PCD format."""
        do_finalize(artifact_dir_pcd, "PCD")

        # Thumbnail may fail in headless mode
        thumbnail_path = os.path.join(artifact_dir_pcd, "thumbnail.png")
        if not os.path.exists(thumbnail_path):
            print("Warning: thumbnail.png not generated (headless mode)")

        # Metadata should always be generated
        assert os.path.exists(os.path.join(artifact_dir_pcd, "map_metadata.json"))

        with open(os.path.join(artifact_dir_pcd, "map_metadata.json")) as f:
            metadata = json.load(f)
        assert metadata["num_points"] == 500


class TestLoadPointcloud:
    """Additional tests for load_pointcloud function."""

    def test_load_pcd_file(self):
        """Test loading PCD file."""
        with tempfile.NamedTemporaryFile(suffix=".pcd", delete=False) as f:
            filepath = f.name

        try:
            expected_points = create_mock_pcd_file(filepath, num_points=100)
            loaded_points = load_pointcloud(filepath)

            assert len(loaded_points) == 100
            # Points may be reordered, check shape
            assert loaded_points.shape == (100, 3)
        finally:
            os.unlink(filepath)

    def test_load_unsupported_format(self):
        """Test loading unsupported file format."""
        with tempfile.NamedTemporaryFile(suffix=".xyz", delete=False) as f:
            f.write(b"1.0 2.0 3.0\n")
            filepath = f.name

        try:
            with pytest.raises(ValueError, match="Unsupported file format"):
                load_pointcloud(filepath)
        finally:
            os.unlink(filepath)


class TestCalculateAreaVolume:
    """Tests for calculate_area_volume function."""

    def test_calculate_area_volume_box(self):
        """Test area/volume calculation for a box-like point cloud."""
        # Create a 10x10x5 box of points
        x = np.linspace(0, 10, 10)
        y = np.linspace(0, 10, 10)
        z = np.linspace(0, 5, 5)

        xx, yy, zz = np.meshgrid(x, y, z)
        points = np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()])

        area, volume = calculate_area_volume(points)

        # Area should be approximately 10 * 10 = 100
        assert 80 < area < 120, f"Area {area} should be ~100"

        # Volume should be approximately 10 * 10 * 5 = 500
        assert 400 < volume < 600, f"Volume {volume} should be ~500"

    def test_calculate_area_volume_empty(self):
        """Test area/volume calculation with empty point cloud."""
        points = np.array([]).reshape(0, 3)
        result = calculate_area_volume(points)
        assert result == 0.0 or result == (0.0, 0.0)
