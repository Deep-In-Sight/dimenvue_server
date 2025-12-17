#!/usr/bin/env python3
"""
Mapping finalization script.
Processes the recorded point cloud to generate:
1. Thumbnail image (with JET colormap on Z-axis)
2. Map metadata (file size, area in square meters)

Uses PDAL for point cloud processing (supports PLY, PCD, LAS, LAZ)
"""

import argparse
import json
import os
import sys
import numpy as np
import laspy
import open3d as o3d
import matplotlib.pyplot as plt
import matplotlib.cm as cm


def load_pointcloud(file_path):
    """
    Load point cloud data from various file formats.
    Supported formats: PLY, PCD, LAS, LAZ
    Returns:
        numpy of shape [N,3] with XYZ coordinates
    """
    file_ext = os.path.splitext(file_path)[1].lower()

    try:
        if file_ext in ['.ply', '.pcd']:
            # Load PLY and PCD files using Open3D
            pcd = o3d.io.read_point_cloud(file_path)
            points = np.asarray(pcd.points)

        elif file_ext in ['.las', '.laz']:
            # Load LAS/LAZ file using laspy
            las = laspy.read(file_path)
            points = np.vstack([las.x, las.y, las.z]).T

        else:
            raise ValueError(f"Unsupported file format: {file_ext}")

        if len(points) == 0:
            raise ValueError(f"No points loaded from {file_path}")

        print(f"Loaded {len(points)} points from {file_path}")
        return points

    except Exception as e:
        print(f"Error loading point cloud {file_path}: {e}")
        raise e


def generate_thumbnail(points, output_path, width=640, height=480):
    """
    Generate thumbnail of the point cloud. (+z up)
    Look at the center from an angle at a distance.
    Apply JET colormap based on Z values.
    """
    try:
        if len(points) == 0:
            print("Warning: Empty point cloud, cannot generate thumbnail")
            return False

        # Use Open3D for visualization and rendering
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Normalize Z values for colormap
        z_values = points[:, 2]
        z_min, z_max = z_values.min(), z_values.max()

        if z_max > z_min:
            # Normalize Z values to [0, 1] for JET colormap
            normalized_z = (z_values - z_min) / (z_max - z_min)

            # Apply JET colormap
            colors = cm.jet(normalized_z)[:, :3]  # Remove alpha channel
            pcd.colors = o3d.utility.Vector3dVector(colors)
        else:
            # If all Z values are the same, use a default color
            pcd.paint_uniform_color([0.7, 0.7, 0.7])

        # Use OffscreenRenderer for thumbnail generation
        renderer = o3d.visualization.rendering.OffscreenRenderer(width, height)

        # Compute bounding box to position camera appropriately
        bbox = pcd.get_axis_aligned_bounding_box()
        center = bbox.get_center()
        extent = bbox.get_extent()
        max_extent = max(extent)

        # Set up the scene
        renderer.scene.add_geometry(
            "pointcloud", pcd, o3d.visualization.rendering.MaterialRecord())

        # Set camera position (looking from above at an angle)
        camera_distance = max_extent * 2.5
        camera_pos = center + camera_distance * np.array([0.5, 0.5, 1.0])

        # Set up camera parameters
        renderer.scene.camera.look_at(center, camera_pos, [0, 0, 1])  # Z up

        # Set field of view to fit the point cloud nicely
        fov = 60.0  # degrees
        renderer.scene.camera.set_projection(
            fov, width / height, 0.1, camera_distance * 3)

        # Render and save the image
        image = renderer.render_to_image()
        o3d.io.write_image(output_path, image)

        print(f"Thumbnail generated: {output_path}")
        return True

    except Exception as e:
        print(f"Error generating thumbnail: {e}")


def calculate_area_volume(points):
    """
    Calculate the area covered by the point cloud in square meters.
    Take the OBB of the XY projection and compute its area.
    """
    try:
        if len(points) == 0:
            print("Warning: Empty point cloud, cannot calculate area")
            return 0.0

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        obb = pcd.get_oriented_bounding_box()

        extent = obb.extent

        area = extent[0] * extent[1]
        volume = extent[0] * extent[1] * extent[2]

        return float(area), float(volume)

    except Exception as e:
        print(f"Error calculating area with Open3D, trying fallback: {e}")


def calculate_tree_size(dir_path):
    """
    Get total size of files in a directory without recursive walking.
    Uses efficient methods for better performance.
    """
    try:
        # Method 1: Try using du command (most efficient on Unix systems)
        import subprocess
        result = subprocess.run(
            ['du', '-sb', dir_path],
            capture_output=True,
            text=True,
            timeout=10,
            check=False
        )
        if result.returncode == 0:
            # du -sb returns size in bytes followed by directory name
            size_bytes = int(result.stdout.split()[0])
            print(f"Directory size (du): {size_bytes} bytes")
            return size_bytes
    except (subprocess.SubprocessError, subprocess.TimeoutExpired, FileNotFoundError, ValueError):
        print("du command failed, exception: ")


def do_finalize(map_dir, map_ext):
    """
    Finalize mapping results by generating thumbnail and metadata.
    Args:
        map_dir: Directory containing mapping artifacts
        map_ext: Point cloud file format (ply, pcd, las, laz)
    Generates:
        - map_dir/thumbnail.png
        - map_dir/map_metadata.json
    """

    # Find the map result file
    map_file = os.path.join(map_dir, f"map_result.{map_ext}")

    if not os.path.exists(map_file):
        print(f"Error: Map file not found: {map_file}")
        sys.exit(1)

    print(f"Processing map file: {map_file}")

    # Load point cloud
    points = load_pointcloud(map_file)

    # Generate thumbnail
    thumbnail_path = os.path.join(map_dir, "thumbnail.png")
    generate_thumbnail(points, thumbnail_path)

    # Calculate metadata
    total_data_size = calculate_tree_size(map_dir)
    area, volume = calculate_area_volume(points)

    # Write metadata
    metadata = {
        "data_size": total_data_size,
        "area": area,
        "volume": volume,
        "num_points": len(points)
    }

    metadata_path = os.path.join(map_dir, "map_metadata.json")
    try:
        with open(metadata_path, 'w', encoding='utf-8') as f:
            json.dump(metadata, f, indent=2)
        print(f"Metadata saved to: {metadata_path}")
    except Exception as e:
        print(f"Error writing metadata: {e}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Finalize mapping results')
    parser.add_argument(
        'artifact_dir', help='Directory containing mapping artifacts')
    parser.add_argument('--file-format', default='ply',
                        help='Point cloud file format (ply, pcd, las, laz)')

    args = parser.parse_args()

    artifact_dir = args.artifact_dir
    file_format = args.file_format

    do_finalize(artifact_dir, file_format)
