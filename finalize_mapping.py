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
import time
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


def remove_outliers(points, nb_neighbors=20, std_ratio=2.0):
    """
    Remove statistical outliers from point cloud.

    Args:
        points: numpy array of shape [N, 3]
        nb_neighbors: number of neighbors to analyze for each point
        std_ratio: standard deviation ratio threshold

    Returns:
        filtered points as numpy array
    """
    if len(points) < nb_neighbors:
        return points

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Statistical outlier removal
    filtered_pcd, inlier_indices = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio
    )

    filtered_points = np.asarray(filtered_pcd.points)
    removed_count = len(points) - len(filtered_points)
    if removed_count > 0:
        print(f"Removed {removed_count} outliers ({100*removed_count/len(points):.1f}%)")

    return filtered_points


def generate_thumbnail(points, output_path, width=640, height=480, max_points=500000):
    """
    Generate thumbnail of the point cloud. (+z up)
    Look at the center from an angle at a distance.
    Apply JET colormap based on Z values.

    Args:
        points: numpy array of shape [N, 3]
        output_path: path to save thumbnail
        width, height: thumbnail dimensions
        max_points: maximum points to render (downsample if exceeded)
    """
    try:
        if len(points) == 0:
            print("Warning: Empty point cloud, cannot generate thumbnail")
            return False

        # Downsample if too many points (for faster outlier removal and rendering)
        if len(points) > max_points:
            print(f"Downsampling from {len(points)} to {max_points} points for thumbnail")
            indices = np.random.choice(len(points), max_points, replace=False)
            points = points[indices]

        # Remove outliers for better visualization
        points = remove_outliers(points, nb_neighbors=20, std_ratio=0.8)

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

        # Set dark background to match dark theme
        renderer.scene.set_background([0.07, 0.09, 0.13, 1.0])  # #121820

        # Compute bounding box to position camera appropriately
        bbox = pcd.get_axis_aligned_bounding_box()
        center = bbox.get_center()
        extent = bbox.get_extent()
        max_extent = max(extent)

        # Get oriented bounding box for camera alignment
        obb = pcd.get_oriented_bounding_box()

        # Set up the scene
        renderer.scene.add_geometry(
            "pointcloud", pcd, o3d.visualization.rendering.MaterialRecord())

        obb_center = obb.center
        obb_extent = obb.extent
        obb_R = obb.R  # Rotation matrix (columns are OBB axes)

        # OBB extent is [x, y, z] in OBB local frame
        # Assuming OBB is flat on XY plane, Z is smallest (height)
        # Camera looks down from above (along -Z of OBB)

        # Calculate camera distance to fit OBB in frame
        # Vertical FOV, so we need to fit the shorter OBB axis vertically
        # and longer axis horizontally (accounting for aspect ratio)
        fov = 60.0  # degrees
        aspect_ratio = width / height

        # Shorter axis goes vertical, longer goes horizontal
        if obb_extent[0] < obb_extent[1]:
            vertical_extent = obb_extent[0]
            horizontal_extent = obb_extent[1]
        else:
            vertical_extent = obb_extent[1]
            horizontal_extent = obb_extent[0]

        # Check which dimension is limiting (vertical or horizontal)
        # For vertical FOV: distance = (extent/2) / tan(fov/2)
        # For horizontal: distance = (extent/2) / tan(fov/2 * aspect_ratio) -- approximate
        fov_rad = np.radians(fov)
        dist_for_vertical = (vertical_extent / 2) / np.tan(fov_rad / 2)
        dist_for_horizontal = (horizontal_extent / 2) / np.tan(fov_rad / 2 * aspect_ratio)

        camera_distance = max(dist_for_vertical, dist_for_horizontal) * 1.1  # 10% margin

        # Camera position: above the center, looking down along OBB's Z axis
        obb_z_axis = obb_R[:, 2]  # Third column is Z axis
        camera_pos = obb_center + camera_distance * obb_z_axis

        # Up vector: use the shorter of X or Y axis of OBB
        # This aligns the shorter dimension with vertical in the image
        obb_x_axis = obb_R[:, 0]
        obb_y_axis = obb_R[:, 1]
        if obb_extent[0] < obb_extent[1]:
            up_vector = obb_x_axis  # X is shorter, use as up
        else:
            up_vector = obb_y_axis  # Y is shorter, use as up

        # Set up camera parameters
        renderer.scene.camera.look_at(obb_center, camera_pos, up_vector)
        renderer.scene.camera.set_projection(
            fov, width / height, 0.1, camera_distance * 4,
            o3d.visualization.rendering.Camera.FovType.Vertical)

        # Render and save the image
        image = renderer.render_to_image()
        o3d.io.write_image(output_path, image)

        print(f"Thumbnail generated: {output_path}")
        return True

    except Exception as e:
        print(f"Error generating thumbnail: {e}")


def generate_preview_cloud(points, output_path, voxel_size=0.1):
    """
    Generate a voxelized preview point cloud for faster loading in viewer.

    Args:
        points: numpy array of shape [N, 3]
        output_path: path to save preview cloud
        voxel_size: voxel size in meters for downsampling
    """
    try:
        if len(points) == 0:
            print("Warning: Empty point cloud, cannot generate preview")
            return False

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Voxel downsample
        pcd_down = pcd.voxel_down_sample(voxel_size)

        # Remove outliers from preview
        pcd_clean, _ = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.8)

        # Get the downsampled points
        preview_points = np.asarray(pcd_clean.points)

        # Save to file - use laspy for LAS/LAZ, Open3D for others
        file_ext = os.path.splitext(output_path)[1].lower()
        if file_ext in ['.las', '.laz']:
            # Use laspy for LAS/LAZ
            header = laspy.LasHeader(point_format=0, version="1.2")
            header.scales = [0.001, 0.001, 0.001]
            header.offsets = [0.0, 0.0, 0.0]
            las = laspy.LasData(header)
            las.x = preview_points[:, 0]
            las.y = preview_points[:, 1]
            las.z = preview_points[:, 2]
            las.write(output_path)
        else:
            o3d.io.write_point_cloud(output_path, pcd_clean)

        original_count = len(points)
        preview_count = len(preview_points)
        print(f"Preview cloud generated: {output_path}")
        print(f"  Original: {original_count:,} points -> Preview: {preview_count:,} points ({100*preview_count/original_count:.1f}%)")

        return True

    except Exception as e:
        print(f"Error generating preview cloud: {e}")
        return False


def calculate_area_volume(points, max_points=100000):
    """
    Calculate the area covered by the point cloud in square meters.
    Take the OBB of the XY projection and compute its area.

    Args:
        points: numpy array of shape [N, 3]
        max_points: maximum points for OBB calculation (downsample if exceeded)
    """
    try:
        if len(points) == 0:
            print("Warning: Empty point cloud, cannot calculate area")
            return 0.0

        # Downsample for faster processing
        if len(points) > max_points:
            indices = np.random.choice(len(points), max_points, replace=False)
            points = points[indices]

        # Remove outliers for accurate area calculation (aggressive)
        points = remove_outliers(points, nb_neighbors=20, std_ratio=0.8)

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


def do_finalize(map_dir, map_ext, preview_voxel_size=0.05):
    """
    Finalize mapping results by generating thumbnail and metadata.
    Args:
        map_dir: Directory containing mapping artifacts
        map_ext: Point cloud file format (ply, pcd, las, laz)
        preview_voxel_size: Voxel size in meters for preview cloud (default 0.05m = 5cm)
    Generates:
        - map_dir/thumbnail.png
        - map_dir/map_result_preview.<map_ext>
        - map_dir/map_metadata.json
    """
    total_start = time.time()

    # Find the map result file
    map_file = os.path.join(map_dir, f"map_result.{map_ext}")

    if not os.path.exists(map_file):
        print(f"Error: Map file not found: {map_file}")
        sys.exit(1)

    print(f"Processing map file: {map_file}")

    # Load point cloud
    t0 = time.time()
    points = load_pointcloud(map_file)
    print(f"[PROFILE] load_pointcloud: {time.time() - t0:.2f}s")

    # Generate thumbnail
    t0 = time.time()
    thumbnail_path = os.path.join(map_dir, "thumbnail.jpg")
    generate_thumbnail(points, thumbnail_path)
    print(f"[PROFILE] generate_thumbnail: {time.time() - t0:.2f}s")

    # Generate preview cloud (voxelized for fast loading)
    t0 = time.time()
    preview_path = os.path.join(map_dir, f"map_result_preview.{map_ext}")
    generate_preview_cloud(points, preview_path, voxel_size=preview_voxel_size)
    print(f"[PROFILE] generate_preview_cloud: {time.time() - t0:.2f}s")

    # Calculate metadata
    t0 = time.time()
    total_data_size = calculate_tree_size(map_dir)
    print(f"[PROFILE] calculate_tree_size: {time.time() - t0:.2f}s")

    t0 = time.time()
    area, volume = calculate_area_volume(points)
    print(f"[PROFILE] calculate_area_volume: {time.time() - t0:.2f}s")

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

    print(f"[PROFILE] Total finalization: {time.time() - total_start:.2f}s")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Finalize mapping results')
    parser.add_argument(
        'artifact_dir', help='Directory containing mapping artifacts')
    parser.add_argument('--file-format', default='ply',
                        help='Point cloud file format (ply, pcd, las, laz)')
    parser.add_argument('--voxel-size', type=float, default=0.1,
                        help='Voxel size in meters for preview cloud (default: 0.1)')

    args = parser.parse_args()

    do_finalize(args.artifact_dir, args.file_format, args.voxel_size)
