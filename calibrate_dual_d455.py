#!/usr/bin/env python3
"""
Dual RealSense D455 Calibration Tool
=====================================
Captures frames from two connected RealSense cameras, uses Open3D's
RANSAC global registration + ICP refinement to compute the rigid
transform from Camera B -> Camera A, and saves it as
DualD455Extrinsics.json for the Unity project.

Usage:
    python calibrate_dual_d455.py [--visualize]

Requirements:
    pip install pyrealsense2 open3d numpy scipy
"""

import argparse
import json
import sys
import time
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    print("ERROR: pyrealsense2 not found. Install with: pip install pyrealsense2")
    sys.exit(1)

try:
    import open3d as o3d
except ImportError:
    print("ERROR: open3d not found. Install with: pip install open3d")
    sys.exit(1)

from scipy.spatial.transform import Rotation

# ---------------------------------------------------------------------------
# Camera capture
# ---------------------------------------------------------------------------

def discover_cameras():
    """Find all connected RealSense devices and return their serial numbers."""
    ctx = rs.context()
    devices = ctx.query_devices()
    serials = []
    for dev in devices:
        serial = dev.get_info(rs.camera_info.serial_number)
        name = dev.get_info(rs.camera_info.name)
        print(f"  Found: {name}  serial={serial}")
        serials.append(serial)
    return serials


def start_pipeline(serial, width=640, height=480, fps=30):
    """Start a RealSense pipeline for the given serial number."""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    return pipeline, profile, depth_scale


def capture_rgbd(pipeline, profile, depth_scale, num_frames=5):
    """
    Capture and average several frames, returning an Open3D PointCloud
    with colors.  Aligns depth to color before projection.
    """
    align = rs.align(rs.stream.color)
    pc = rs.pointcloud()

    # Let auto-exposure stabilize
    for _ in range(30):
        pipeline.wait_for_frames()

    # Accumulate frames for noise reduction
    depth_accum = None
    color_last = None
    for _ in range(num_frames):
        frameset = pipeline.wait_for_frames()
        aligned = align.process(frameset)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data()).astype(np.float64)
        if depth_accum is None:
            depth_accum = depth_image
        else:
            depth_accum += depth_image
        color_last = np.asanyarray(color_frame.get_data())

    if depth_accum is None or color_last is None:
        return None, None

    depth_avg = (depth_accum / num_frames).astype(np.uint16)

    # Get intrinsics from aligned depth stream
    depth_prof = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    intr = depth_prof.get_intrinsics()

    o3d_intr = o3d.camera.PinholeCameraIntrinsic(
        intr.width, intr.height,
        intr.fx, intr.fy,
        intr.ppx, intr.ppy
    )

    o3d_depth_scale = 1.0 / depth_scale
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(color_last),
        o3d.geometry.Image(depth_avg),
        depth_scale=o3d_depth_scale,
        depth_trunc=3.0,
        convert_rgb_to_intensity=False
    )

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_intr)

    # Flip Y to convert from RealSense (Y-down) to Unity (Y-up) coordinates
    pts = np.asarray(pcd.points)
    pts[:, 1] *= -1.0
    pcd.points = o3d.utility.Vector3dVector(pts)

    return pcd, color_last


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------

def preprocess(pcd, voxel_size):
    """Downsample, estimate normals, compute FPFH features."""
    down = pcd.voxel_down_sample(voxel_size)
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0, max_nn=30)
    )
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5.0, max_nn=100)
    )
    return down, fpfh


def global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    """RANSAC-based global registration using FPFH features."""
    distance_threshold = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down,
        source_fpfh, target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)
    )
    return result


def refine_registration(source, target, initial_transform, voxel_size):
    """Refine with point-to-plane ICP, then colored ICP."""
    # Point-to-plane ICP (coarse)
    source.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0, max_nn=30)
    )
    target.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0, max_nn=30)
    )

    result_icp = o3d.pipelines.registration.registration_icp(
        source, target,
        max_correspondence_distance=voxel_size * 1.0,
        init=initial_transform,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
    )

    # Colored ICP (fine refinement using both geometry and color)
    try:
        result_color = o3d.pipelines.registration.registration_colored_icp(
            source, target,
            max_correspondence_distance=voxel_size * 0.5,
            init=result_icp.transformation,
            estimation_method=o3d.pipelines.registration.TransformationEstimationForColoredICP(
                lambda_geometric=0.968
            ),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
        )
        return result_color
    except Exception:
        return result_icp


def compute_registration(pcd_a, pcd_b, voxel_size=0.02):
    """
    Full registration pipeline: RANSAC global + ICP refinement.
    Returns the 4x4 transform that maps points from B-space to A-space.
    """
    print(f"\n--- Registration (voxel_size={voxel_size}m) ---")
    print(f"  Cloud A: {len(pcd_a.points)} points")
    print(f"  Cloud B: {len(pcd_b.points)} points")

    # Downsample for global registration
    coarse_voxel = voxel_size * 2.0
    print(f"  Downsampling for FPFH (voxel={coarse_voxel:.3f}m)...")
    a_down, a_fpfh = preprocess(pcd_a, coarse_voxel)
    b_down, b_fpfh = preprocess(pcd_b, coarse_voxel)
    print(f"  Downsampled: A={len(a_down.points)}, B={len(b_down.points)} points")

    # RANSAC global registration
    print("  Running RANSAC global registration...")
    result_ransac = global_registration(pcd_b, pcd_a, b_fpfh, a_fpfh, coarse_voxel)
    print(f"  RANSAC fitness={result_ransac.fitness:.4f}, RMSE={result_ransac.inlier_rmse:.4f}m")
    print(f"  RANSAC correspondences: {len(result_ransac.correspondence_set)}")

    if result_ransac.fitness < 0.05:
        print("  WARNING: RANSAC fitness very low. Trying with larger voxel...")
        coarse_voxel = voxel_size * 4.0
        a_down2, a_fpfh2 = preprocess(pcd_a, coarse_voxel)
        b_down2, b_fpfh2 = preprocess(pcd_b, coarse_voxel)
        result_ransac = global_registration(pcd_b, pcd_a, b_fpfh2, a_fpfh2, coarse_voxel)
        print(f"  Retry RANSAC fitness={result_ransac.fitness:.4f}, RMSE={result_ransac.inlier_rmse:.4f}m")

    # Downsample for ICP refinement
    print(f"  Downsampling for ICP (voxel={voxel_size:.3f}m)...")
    a_fine = pcd_a.voxel_down_sample(voxel_size)
    b_fine = pcd_b.voxel_down_sample(voxel_size)

    # ICP refinement
    print("  Running ICP refinement (point-to-plane + colored)...")
    result_icp = refine_registration(b_fine, a_fine, result_ransac.transformation, voxel_size)
    print(f"  ICP fitness={result_icp.fitness:.4f}, RMSE={result_icp.inlier_rmse:.4f}m")

    return result_icp.transformation, result_icp.fitness, result_icp.inlier_rmse


# ---------------------------------------------------------------------------
# Transform conversion
# ---------------------------------------------------------------------------

def matrix_to_unity_euler(T):
    """
    Convert a 4x4 rigid transform to Unity-compatible translation + euler.
    Returns (tx, ty, tz, rx, ry, rz) where euler angles are in degrees
    matching Unity's Quaternion.Euler(rx, ry, rz) convention (intrinsic YXZ).
    """
    R = T[:3, :3]
    t = T[:3, 3]

    rot = Rotation.from_matrix(R)
    # Unity's Quaternion.Euler applies intrinsic Y-X-Z order
    euler_yxz = rot.as_euler('YXZ', degrees=True)  # returns [y, x, z]

    rx = float(euler_yxz[1])
    ry = float(euler_yxz[0])
    rz = float(euler_yxz[2])

    return float(t[0]), float(t[1]), float(t[2]), rx, ry, rz


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Calibrate two RealSense D455 cameras")
    parser.add_argument("--visualize", action="store_true", help="Show point clouds before/after alignment")
    parser.add_argument("--voxel", type=float, default=0.01, help="Voxel size for ICP refinement (meters)")
    parser.add_argument("--output", default="DualD455Extrinsics.json", help="Output JSON file path")
    args = parser.parse_args()

    print("=" * 60)
    print("  Dual RealSense D455 Calibration Tool")
    print("=" * 60)

    # Discover cameras
    print("\nSearching for RealSense cameras...")
    serials = discover_cameras()
    if len(serials) < 2:
        print(f"\nERROR: Need 2 cameras, found {len(serials)}. "
              "Make sure both cameras are connected via USB 3.")
        sys.exit(1)

    serial_a = serials[0]
    serial_b = serials[1]
    print(f"\n  Camera A (reference): {serial_a}")
    print(f"  Camera B (to align):  {serial_b}")

    # Start both cameras
    print("\nStarting cameras...")
    pipe_a, prof_a, scale_a = start_pipeline(serial_a)
    time.sleep(1)
    pipe_b, prof_b, scale_b = start_pipeline(serial_b)

    print("  Waiting for auto-exposure to stabilize...")
    time.sleep(2)

    # Capture point clouds
    print("\nCapturing from Camera A...")
    pcd_a, color_a = capture_rgbd(pipe_a, prof_a, scale_a, num_frames=5)
    print(f"  Got {len(pcd_a.points)} points")

    print("Capturing from Camera B...")
    pcd_b, color_b = capture_rgbd(pipe_b, prof_b, scale_b, num_frames=5)
    print(f"  Got {len(pcd_b.points)} points")

    # Stop cameras
    pipe_a.stop()
    pipe_b.stop()

    if pcd_a is None or pcd_b is None:
        print("\nERROR: Failed to capture point clouds.")
        sys.exit(1)

    # Remove statistical outliers
    print("\nFiltering outliers...")
    pcd_a, _ = pcd_a.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd_b, _ = pcd_b.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print(f"  After filtering: A={len(pcd_a.points)}, B={len(pcd_b.points)}")

    # Optional: show before alignment
    if args.visualize:
        print("\n[Visualization] Showing BEFORE alignment (close window to continue)...")
        pcd_a_vis = o3d.geometry.PointCloud(pcd_a)
        pcd_b_vis = o3d.geometry.PointCloud(pcd_b)
        o3d.visualization.draw_geometries(
            [pcd_a_vis, pcd_b_vis],
            window_name="Before Alignment (A=original colors, B=original colors)"
        )

    # Run registration
    T, fitness, rmse = compute_registration(pcd_a, pcd_b, voxel_size=args.voxel)

    print(f"\n--- Final Transform (B -> A) ---")
    print(T)

    # Convert to Unity format
    tx, ty, tz, rx, ry, rz = matrix_to_unity_euler(T)
    print(f"\n  Unity Translation: ({tx:.4f}, {ty:.4f}, {tz:.4f})")
    print(f"  Unity Rotation:    ({rx:.2f}, {ry:.2f}, {rz:.2f}) degrees")

    # Save JSON
    output = {
        "tx": tx, "ty": ty, "tz": tz,
        "rx": rx, "ry": ry, "rz": rz,
        "cameraASerial": serial_a,
        "cameraBSerial": serial_b,
        "method": "open3d_ransac_icp",
        "fitness": fitness,
        "rmse": rmse
    }

    with open(args.output, 'w') as f:
        json.dump(output, f, indent=4)
    print(f"\n  Saved to {args.output}")

    # Optional: show after alignment
    if args.visualize:
        print("\n[Visualization] Showing AFTER alignment (close window to continue)...")
        pcd_b_aligned = o3d.geometry.PointCloud(pcd_b)
        pcd_b_aligned.transform(T)
        pcd_a.paint_uniform_color([1, 0, 0])    # Red for A
        pcd_b_aligned.paint_uniform_color([0, 0, 1])  # Blue for B
        o3d.visualization.draw_geometries(
            [pcd_a, pcd_b_aligned],
            window_name="After Alignment (Red=A, Blue=B)"
        )

    print("\n  Done! Load this calibration in Unity (it loads on play).")
    print("=" * 60)


if __name__ == "__main__":
    main()
