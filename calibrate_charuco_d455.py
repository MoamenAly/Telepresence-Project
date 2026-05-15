#!/usr/bin/env python3
"""
Dual RealSense D455 Charuco Extrinsic Calibration
=================================================
Computes the rigid transform from Camera B -> Camera A by observing a
Charuco board with both cameras simultaneously, then writes
DualD455Extrinsics.json in the schema the Unity project loads on Play.

Re-run any time the physical camera setup changes.

Workflow:
    # One-time: generate a printable board (default: 5x7 squares, 40mm,
    # 30mm markers, DICT_5X5_100). Print it on rigid material.
    python calibrate_charuco_d455.py --generate-board

    # Each calibration:
    python calibrate_charuco_d455.py

    # Hold the board so BOTH cameras can see it. In the preview window:
    #   SPACE = capture sample,  ENTER = finish,  R = reset,  ESC = cancel.
    # Aim for 8-12 samples from slightly different board poses.

    # Or headless (auto-capture every ~0.5s, no preview):
    python calibrate_charuco_d455.py --no-preview --samples 12

Run this from the Unity project root so the output JSON lands in the
right place (the Unity loader looks at <projectRoot>/DualD455Extrinsics.json).

Requirements:
    pip install pyrealsense2 opencv-contrib-python numpy scipy
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:
    print("ERROR: opencv-contrib-python not found.")
    print("    Install with: pip install opencv-contrib-python")
    sys.exit(1)

if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "CharucoDetector"):
    print("ERROR: cv2.aruco.CharucoDetector not available.")
    print("    You need OpenCV >= 4.7 with the contrib modules.")
    print("    Reinstall:  pip install --upgrade opencv-contrib-python")
    sys.exit(1)

try:
    import pyrealsense2 as rs
except ImportError:
    print("ERROR: pyrealsense2 not found. Install with: pip install pyrealsense2")
    sys.exit(1)

from scipy.spatial.transform import Rotation


# Map dictionary name -> OpenCV constant
DICTIONARY_NAMES = {
    "DICT_4X4_50":   cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100":  cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250":  cv2.aruco.DICT_4X4_250,
    "DICT_5X5_100":  cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250":  cv2.aruco.DICT_5X5_250,
    "DICT_6X6_250":  cv2.aruco.DICT_6X6_250,
}


# ---------------------------------------------------------------------------
# Board and detector
# ---------------------------------------------------------------------------

def make_board(squares_x, squares_y, square_length, marker_length, dict_name):
    if dict_name not in DICTIONARY_NAMES:
        raise ValueError(f"Unknown dictionary '{dict_name}'. "
                         f"Choose from: {list(DICTIONARY_NAMES.keys())}")
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY_NAMES[dict_name])
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y),
        square_length,
        marker_length,
        dictionary,
    )
    return board


def make_detector(board):
    detector_params = cv2.aruco.DetectorParameters()
    charuco_params = cv2.aruco.CharucoParameters()
    return cv2.aruco.CharucoDetector(board, charuco_params, detector_params)


def generate_board_image(board, output_path, squares_x, squares_y, dpi):
    # OpenCV's generateImage takes a pixel size; we derive it from physical
    # board size + DPI so the printed result matches the configured square
    # length exactly.
    width_m = squares_x * board.getSquareLength()
    height_m = squares_y * board.getSquareLength()
    ppm = dpi / 0.0254  # pixels per meter
    pixels_x = int(round(width_m * ppm))
    pixels_y = int(round(height_m * ppm))

    img = board.generateImage((pixels_x, pixels_y), marginSize=20, borderBits=1)
    cv2.imwrite(str(output_path), img)
    print(f"Saved Charuco board to {output_path}")
    print(f"  Grid: {squares_x}x{squares_y} squares")
    print(f"  Square length: {board.getSquareLength()*1000:.1f} mm")
    print(f"  Marker length: {board.getMarkerLength()*1000:.1f} mm")
    print(f"  Image size:    {pixels_x}x{pixels_y} px at {dpi} DPI")
    print(f"  Physical size: {width_m*100:.1f}x{height_m*100:.1f} cm")
    print()
    print("  IMPORTANT: print this at 100% scale (no 'fit to page'), then")
    print("  measure one square with a ruler and confirm it matches.")


# ---------------------------------------------------------------------------
# RealSense capture
# ---------------------------------------------------------------------------

def discover_cameras():
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
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    profile = pipeline.start(config)
    return pipeline, profile


def get_color_intrinsics(profile):
    color_prof = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_prof.get_intrinsics()

    K = np.array([
        [intr.fx, 0.0,     intr.ppx],
        [0.0,     intr.fy, intr.ppy],
        [0.0,     0.0,     1.0     ],
    ], dtype=np.float64)
    # D455 color uses Brown-Conrady: [k1, k2, p1, p2, k3]
    dist = np.array(intr.coeffs, dtype=np.float64)
    return K, dist


def grab_color_frame(pipeline):
    frames = pipeline.wait_for_frames()
    color = frames.get_color_frame()
    if not color:
        return None
    return np.asanyarray(color.get_data())


# ---------------------------------------------------------------------------
# Charuco detection + pose
# ---------------------------------------------------------------------------

def detect_and_pose(image_bgr, board, detector, K, dist, min_corners=8):
    """
    Returns:
        R (3x3) or None
        t (3,) or None  - board origin expressed in camera coords (meters)
        info dict with everything needed to draw an overlay
    """
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)

    info = {
        "charuco_corners": charuco_corners,
        "charuco_ids": charuco_ids,
        "marker_corners": marker_corners,
        "marker_ids": marker_ids,
        "n_corners": 0,
    }

    if charuco_corners is None or charuco_ids is None:
        return None, None, info
    info["n_corners"] = int(len(charuco_corners))
    if info["n_corners"] < min_corners:
        return None, None, info

    obj_pts, img_pts = board.matchImagePoints(charuco_corners, charuco_ids)
    if obj_pts is None or len(obj_pts) < 4:
        return None, None, info

    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, None, info

    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.flatten(), info


def draw_overlay(image, R, t, K, dist, info, label):
    out = image.copy()

    if info["marker_corners"] is not None and info["marker_ids"] is not None \
            and len(info["marker_corners"]) > 0:
        cv2.aruco.drawDetectedMarkers(out, info["marker_corners"], info["marker_ids"])
    if info["charuco_corners"] is not None and info["charuco_ids"] is not None \
            and len(info["charuco_corners"]) > 0:
        cv2.aruco.drawDetectedCornersCharuco(out, info["charuco_corners"],
                                             info["charuco_ids"], (0, 255, 0))

    if R is not None and t is not None:
        rvec, _ = cv2.Rodrigues(R)
        cv2.drawFrameAxes(out, K, dist, rvec, t.reshape(3, 1), 0.05)
        status = (f"{label}: pose OK   z={t[2]:.2f}m   "
                  f"corners={info['n_corners']}")
        color = (0, 255, 0)
    else:
        status = f"{label}: no pose (corners={info['n_corners']})"
        color = (0, 0, 255)

    cv2.putText(out, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, color, 2, cv2.LINE_AA)
    return out


# ---------------------------------------------------------------------------
# Transform algebra
# ---------------------------------------------------------------------------

def compose_b_to_a(R_a, t_a, R_b, t_b):
    """
    solvePnP gives R_board->cam, t_board->cam such that
        p_cam = R @ p_board + t

    For each camera we have T_board_to_A and T_board_to_B.
    p_A = T_board_to_A @ p_board
    p_B = T_board_to_B @ p_board   =>   p_board = T_board_to_B^-1 @ p_B
    p_A = (T_board_to_A @ T_board_to_B^-1) @ p_B

    Therefore  T_B_to_A = T_board_to_A @ T_board_to_B^-1, which in (R, t) form is:
        R_BA = R_A @ R_B^T
        t_BA = t_A - R_BA @ t_B
    """
    R_ba = R_a @ R_b.T
    t_ba = t_a - R_ba @ t_b
    return R_ba, t_ba


# RealSense color frames: X right, Y down, Z forward (right-handed).
# Unity:                  X right, Y up,   Z forward (left-handed).
# Per-axis Y flip:  p_unity = F @ p_rs   where F = diag(1, -1, 1).
# Conjugating a rigid transform:
#   R_unity = F @ R_rs @ F        (F is its own inverse)
#   t_unity = F @ t_rs
# This matches calibrate_dual_d455.py, which flips Y on raw point clouds.
F = np.diag([1.0, -1.0, 1.0])


def to_unity_frame(R, t):
    return F @ R @ F, F @ t


def matrix_to_unity_euler(R, t):
    """
    Returns (tx, ty, tz, rx, ry, rz) where the Euler triple matches
    Unity's Quaternion.Euler(rx, ry, rz) (intrinsic YXZ).
    """
    euler_yxz = Rotation.from_matrix(R).as_euler("YXZ", degrees=True)
    rx, ry, rz = float(euler_yxz[1]), float(euler_yxz[0]), float(euler_yxz[2])
    return float(t[0]), float(t[1]), float(t[2]), rx, ry, rz


# ---------------------------------------------------------------------------
# Sample averaging
# ---------------------------------------------------------------------------

def average_transforms(samples):
    """
    samples: list of (R, t).
    Returns (R_mean, t_mean, t_std_xyz, angle_std_deg).
    """
    Rs = np.array([s[0] for s in samples])
    ts = np.array([s[1] for s in samples])

    R_mean = Rotation.from_matrix(Rs).mean().as_matrix()
    t_mean = ts.mean(axis=0)
    t_std = ts.std(axis=0)

    # Angular distance of each sample from the mean
    deviations_deg = []
    for R in Rs:
        rel = Rotation.from_matrix(R_mean.T @ R)
        deviations_deg.append(np.degrees(rel.magnitude()))
    angle_std_deg = float(np.std(deviations_deg))

    return R_mean, t_mean, t_std, angle_std_deg


# ---------------------------------------------------------------------------
# Main calibration loop
# ---------------------------------------------------------------------------

def run_calibration(args):
    print("\nSearching for RealSense cameras...")
    serials = discover_cameras()
    if len(serials) < 2:
        print(f"\nERROR: Need 2 cameras, found {len(serials)}.")
        sys.exit(1)

    serial_a, serial_b = serials[0], serials[1]
    print(f"\nCamera A (reference): {serial_a}")
    print(f"Camera B (to align):  {serial_b}")

    print("\nStarting color streams...")
    pipe_a, prof_a = start_pipeline(serial_a)
    time.sleep(0.5)
    pipe_b, prof_b = start_pipeline(serial_b)

    K_a, dist_a = get_color_intrinsics(prof_a)
    K_b, dist_b = get_color_intrinsics(prof_b)

    print("Waiting for auto-exposure to stabilize...")
    for _ in range(30):
        pipe_a.wait_for_frames()
        pipe_b.wait_for_frames()

    board = make_board(args.squares_x, args.squares_y,
                       args.square_length, args.marker_length,
                       args.dictionary)
    detector = make_detector(board)

    samples = []

    print("\n" + "=" * 60)
    print("  Hold the Charuco board so BOTH cameras see it clearly.")
    print("  ~0.5-1.5m from each camera, well lit, not motion-blurred.")
    print("  Capture from a few SLIGHTLY different board poses to average out noise.")
    if args.no_preview:
        print(f"  Headless: auto-capturing {args.samples} samples...")
    else:
        print("  Controls (focus the preview window):")
        print("    SPACE = capture sample   ENTER = finish")
        print("    R     = drop all samples ESC   = cancel")
        print(f"  Target: {args.samples} samples.")
    print("=" * 60)

    try:
        last_auto_capture = 0.0
        while True:
            img_a = grab_color_frame(pipe_a)
            img_b = grab_color_frame(pipe_b)
            if img_a is None or img_b is None:
                continue

            R_a, t_a, info_a = detect_and_pose(img_a, board, detector, K_a, dist_a,
                                               min_corners=args.min_corners)
            R_b, t_b, info_b = detect_and_pose(img_b, board, detector, K_b, dist_b,
                                               min_corners=args.min_corners)

            both_ok = (R_a is not None) and (R_b is not None)

            if args.no_preview:
                now = time.time()
                if both_ok and (now - last_auto_capture) >= 0.5:
                    R_ba, t_ba = compose_b_to_a(R_a, t_a, R_b, t_b)
                    samples.append((R_ba, t_ba))
                    last_auto_capture = now
                    print(f"  Captured sample {len(samples)}/{args.samples} "
                          f"(corners A={info_a['n_corners']}, B={info_b['n_corners']})")
                    if len(samples) >= args.samples:
                        break
                elif not both_ok:
                    print(f"  ...waiting (corners A={info_a['n_corners']}, "
                          f"B={info_b['n_corners']})", end="\r")
                continue

            overlay_a = draw_overlay(img_a, R_a, t_a, K_a, dist_a, info_a, "Camera A")
            overlay_b = draw_overlay(img_b, R_b, t_b, K_b, dist_b, info_b, "Camera B")
            combined = np.hstack([overlay_a, overlay_b])
            cv2.putText(combined,
                        f"Samples: {len(samples)}/{args.samples}   "
                        "SPACE=capture  ENTER=finish  R=reset  ESC=cancel",
                        (10, combined.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.imshow("Dual D455 Charuco Calibration", combined)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                print("\nCancelled by user.")
                return None
            elif key == ord(" "):
                if not both_ok:
                    print("  Cannot capture: board not detected in both cameras.")
                else:
                    R_ba, t_ba = compose_b_to_a(R_a, t_a, R_b, t_b)
                    samples.append((R_ba, t_ba))
                    print(f"  Captured sample {len(samples)} "
                          f"(corners A={info_a['n_corners']}, B={info_b['n_corners']})")
            elif key == ord("r") or key == ord("R"):
                samples.clear()
                print("  Samples reset.")
            elif key in (13, 10):  # ENTER (LF/CR)
                if not samples:
                    print("  Need at least 1 sample before finishing.")
                else:
                    break
    finally:
        pipe_a.stop()
        pipe_b.stop()
        cv2.destroyAllWindows()

    if not samples:
        print("\nNo samples captured. Nothing saved.")
        return None

    print(f"\nAveraging {len(samples)} samples...")
    R_ba_rs, t_ba_rs, t_std, angle_std = average_transforms(samples)

    print(f"  Translation std: "
          f"({t_std[0]*1000:.1f}, {t_std[1]*1000:.1f}, {t_std[2]*1000:.1f}) mm")
    print(f"  Rotation std:    {angle_std:.3f} deg")

    if t_std.max() > 0.01 or angle_std > 1.0:
        print("  WARNING: high variance across samples. Consider:")
        print("    - holding the board steadier between captures")
        print("    - capturing in better / more even lighting")
        print("    - placing the board closer and more frontal to each camera")
        print("    - using a more rigid board (foamcore / clipboard, not loose paper)")

    R_ba, t_ba = to_unity_frame(R_ba_rs, t_ba_rs)
    tx, ty, tz, rx, ry, rz = matrix_to_unity_euler(R_ba, t_ba)

    print(f"\n  Unity Translation: ({tx:.4f}, {ty:.4f}, {tz:.4f}) m")
    print(f"  Unity Rotation:    ({rx:.2f}, {ry:.2f}, {rz:.2f}) deg")

    output = {
        "tx": tx, "ty": ty, "tz": tz,
        "rx": rx, "ry": ry, "rz": rz,
        "cameraASerial": serial_a,
        "cameraBSerial": serial_b,
        "method": "charuco_solvepnp",
        "samples": len(samples),
        "translationStdMm": [float(v * 1000) for v in t_std],
        "rotationStdDeg": angle_std,
        "board": {
            "squaresX": args.squares_x,
            "squaresY": args.squares_y,
            "squareLength": args.square_length,
            "markerLength": args.marker_length,
            "dictionary": args.dictionary,
        },
    }

    output_path = Path(args.output)
    output_path.write_text(json.dumps(output, indent=4))
    print(f"\n  Saved to {output_path.resolve()}")
    print("  Press Play in Unity (or hit F-load) to apply the new calibration.")
    return output


def main():
    parser = argparse.ArgumentParser(
        description="Charuco-based extrinsic calibration for dual RealSense D455.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--generate-board", action="store_true",
                        help="Save a printable PNG of the Charuco board and exit")
    parser.add_argument("--board-image", default="charuco_board.png",
                        help="Output path for --generate-board")
    parser.add_argument("--board-dpi", type=int, default=300,
                        help="Print DPI for the generated board image")

    parser.add_argument("--squares-x", type=int, default=5,
                        help="Number of squares along board X (columns)")
    parser.add_argument("--squares-y", type=int, default=7,
                        help="Number of squares along board Y (rows)")
    parser.add_argument("--square-length", type=float, default=0.04,
                        help="Square side length (meters)")
    parser.add_argument("--marker-length", type=float, default=0.03,
                        help="ArUco marker side length (meters); must be < --square-length")
    parser.add_argument("--dictionary", default="DICT_5X5_100",
                        choices=list(DICTIONARY_NAMES.keys()),
                        help="ArUco dictionary")

    parser.add_argument("--samples", type=int, default=10,
                        help="Number of samples to average over")
    parser.add_argument("--min-corners", type=int, default=8,
                        help="Reject frames with fewer detected charuco corners")
    parser.add_argument("--no-preview", action="store_true",
                        help="Run headless with auto-capture (no preview window)")
    parser.add_argument("--output", default="DualD455Extrinsics.json",
                        help="Output JSON path (relative to current working directory)")

    args = parser.parse_args()

    if args.marker_length >= args.square_length:
        print("ERROR: --marker-length must be strictly less than --square-length.")
        sys.exit(1)

    print("=" * 60)
    print("  Dual RealSense D455 Charuco Extrinsic Calibration")
    print("=" * 60)

    if args.generate_board:
        board = make_board(args.squares_x, args.squares_y,
                           args.square_length, args.marker_length,
                           args.dictionary)
        generate_board_image(board, args.board_image,
                             args.squares_x, args.squares_y, args.board_dpi)
        print("=" * 60)
        return

    run_calibration(args)
    print("=" * 60)


if __name__ == "__main__":
    main()
