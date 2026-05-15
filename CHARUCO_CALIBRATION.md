# Charuco Extrinsic Calibration for Dual D455

`calibrate_charuco_d455.py` computes the rigid transform from Camera **B** to Camera **A** by observing a printed Charuco board with both cameras simultaneously, then writes `DualD455Extrinsics.json` in the schema the Unity project already loads on Play.

## What it is for

The Unity dual‑camera pipeline (`RsDualCameraExtrinsicsCalibration` + `RsDualPointCloudFusionManager`) needs to know how Camera B sits relative to Camera A — six numbers (`tx, ty, tz, rx, ry, rz`) — before it can merge the two point clouds into one consistent mesh. If those numbers are wrong, the two clouds render with doubled geometry, ghost edges, and color seams.

This script finds those six numbers from scratch in ~10 seconds whenever the physical camera setup changes — no scene overlap, no textured walls, no manual nudging required.

### Where it fits among the other calibration paths

The repo ships three ways to obtain the extrinsics; they stack rather than replace each other:

| Method | Script / control | When to use |
|---|---|---|
| **Charuco board + solvePnP** | `calibrate_charuco_d455.py` | Setup just changed, or you need a clean reference. **Run this first.** |
| Open3D RANSAC + ICP on natural scene | `calibrate_dual_d455.py` | No board on hand, both cameras share a textured scene |
| In‑engine ICP refinement | F6 in Play mode (`RsDualPointCloudAutoAligner`) | Fine‑tune an already‑close calibration against the live scene |
| Manual nudge | WASDQE + numpad in Play mode (`RsDualCalibrationAssist`) | Last‑mile visual tweaks, save with F5 |

Output of every method ends up in the same `DualD455Extrinsics.json` at the project root, which `RsDualCameraExtrinsicsCalibration.LoadExtrinsics()` reads automatically on Awake.

## Prerequisites

- **Hardware**: 2× Intel RealSense D455, each on its own USB 3 port, rigidly mounted (the calibration is invalidated the instant a camera moves).
- **OS**: Windows or Linux. The script itself is cross‑platform; only the Unity side is Windows‑specific in this repo.
- **Python**: 3.8 or newer.
- **A Charuco board.** You can print one with the script (see below) or use any board you already have.

## Installation

From the project root:

```bash
pip install -r requirements.txt
```

This installs:

- `pyrealsense2` — camera capture
- `opencv-contrib-python>=4.7` — Charuco detector and `matchImagePoints` (the modern API)
- `numpy`, `scipy` — math, rotation averaging
- `open3d` — only used by the other (ICP) calibration script; safe to leave installed

Verify OpenCV has the required Aruco support:

```bash
python -c "import cv2; print(cv2.__version__); print(hasattr(cv2.aruco, 'CharucoDetector'))"
```

Expect `4.7.0` or higher and `True`. If you see `False`, you have plain `opencv-python` installed instead of `opencv-contrib-python` — uninstall the former and reinstall the contrib build.

## Step 1 — Print a Charuco board (one time)

From the project root:

```bash
python calibrate_charuco_d455.py --generate-board
```

Defaults produce `charuco_board.png` configured as:

- 5 × 7 squares
- 40 mm square length, 30 mm marker length
- `DICT_5X5_100` dictionary
- 300 DPI output (~2.4 k × 3.3 k pixels)

That prints onto A3 paper at roughly 20 × 28 cm. After printing:

1. **Disable "Fit to page" / "Scale to fit"** in the print dialog. Print at 100% scale.
2. Measure one square with a ruler. If it isn't exactly 40 mm, either reprint or pass the real value on every subsequent run: `--square-length 0.0395` (whatever it actually came out to).
3. Mount the print on a **rigid backing** — foamcore, mat board, or a clipboard. A floppy sheet introduces calibration error proportional to how much it sags.

### Using a different board

If you already have a Charuco board, pass its parameters:

```bash
python calibrate_charuco_d455.py \
  --squares-x 7 --squares-y 5 \
  --square-length 0.03 --marker-length 0.022 \
  --dictionary DICT_4X4_100
```

Constraint: `--marker-length` must be strictly less than `--square-length`.

## Step 2 — Run calibration

Run from the **project root** so the output lands at `<projectRoot>/DualD455Extrinsics.json` (where the Unity loader reads it from):

```bash
python calibrate_charuco_d455.py
```

A window opens with both camera feeds side by side. Detected markers are outlined, Charuco corners are dotted, and the board's coordinate axes are drawn once a 6‑DoF pose locks. The status bar shows `pose OK / no pose` plus the number of corners detected per camera.

Hold the board so **both cameras see it clearly**, at roughly 0.5–1.5 m from each, well lit, and not motion‑blurred.

| Key | Action |
|---|---|
| **SPACE** | Capture a sample (rejected if board isn't detected in both cameras) |
| **R** | Drop all samples and start over |
| **ENTER** | Finish, average all samples, save the JSON |
| **ESC** | Cancel without saving |

Aim for **8–12 samples** captured from **slightly different board poses** — rotate the board 10–20° between captures, move it 20–30 cm. Different poses cancel out per‑frame solvePnP noise; identical poses don't.

### Headless mode

For scripted / unattended runs (no preview, auto‑capture every ~0.5 s once both cameras have a pose):

```bash
python calibrate_charuco_d455.py --no-preview --samples 12
```

## Step 3 — Verify quality

After ENTER, the script reports:

```
Translation std: (1.2, 0.8, 1.5) mm
Rotation std:    0.18 deg
```

Use these as the trust signal for the result:

| | Good | OK | Bad — recalibrate |
|---|---|---|---|
| Translation std (any axis) | < 3 mm | < 10 mm | > 10 mm |
| Rotation std | < 0.3° | < 1.0° | > 1.0° |

If the script prints `WARNING: high variance across samples`, fix the cause and recapture:

- Hold the board steadier between captures (the std is sensitive to motion blur)
- Even out the lighting (no glare on the board, no underexposure)
- Move the board closer and more frontal to each camera
- Use a more rigid board backing
- Increase `--min-corners` (default 8) to reject borderline detections

## Step 4 — Apply in Unity

The script writes to the path the Unity loader expects. Just press Play on `Main.unity` and watch the Console for:

```
[Extrinsics] Loaded from .../DualD455Extrinsics.json T=(...) R=(...)
```

`RsDevice_B` should jump to the new offset and the two clouds should now line up in the scene view. Optionally press **F6** to let the in‑engine ICP refine against your live scene; press **F5** to save the refined values; **F7** undoes F6.

## Troubleshooting

**"Need 2 cameras, found 0/1"** — USB connection issue. Both D455s must be on USB 3 ports (blue connectors). Run `rs-enumerate-devices` from the RealSense SDK to confirm both are visible at the OS level.

**Window opens but `no pose` on both cameras** — the board can't be seen. Check focus, lighting, distance (0.5–1.5 m), and that the board is fully in frame.

**Window shows pose on one camera but not the other** — that camera doesn't see the board well enough. Move the board so it's clearly visible to both, or rotate it more frontally to the camera that's failing.

**Calibration applied but clouds fly apart instead of aligning** — the camera serials may be swapped between Python and Unity. The JSON's `cameraASerial` must match Unity's `RsDevice_A` `RequestedSerialNumber` (and B with B). `pyrealsense2` enumerates in OS order, which isn't guaranteed to match Unity's `RsDevice` ordering. Fix by either:
1. Swapping the `RequestedSerialNumber` fields on `RsDevice_A` / `RsDevice_B` in `Main.unity`, or
2. Swapping the physical USB ports.

**Calibration quality is fine in the script's report but the clouds still don't merge cleanly in Unity** — likely a *depth* quality issue, not extrinsics. Confirm the RealSense filter chain (decimation → spatial → temporal → hole‑fill) is enabled on both `RsProcessingPipe_A` and `RsProcessingPipe_B` before judging the calibration.

**`AttributeError: module 'cv2.aruco' has no attribute 'CharucoDetector'`** — you have `opencv-python` installed instead of `opencv-contrib-python`. Fix:

```bash
pip uninstall -y opencv-python opencv-python-headless
pip install --upgrade opencv-contrib-python
```

## CLI reference

```text
--generate-board              Save a printable PNG and exit
--board-image PATH            Output path for --generate-board (default: charuco_board.png)
--board-dpi N                 Print DPI for the generated board (default: 300)

--squares-x N                 Columns (default: 5)
--squares-y N                 Rows (default: 7)
--square-length M             Square side length in meters (default: 0.04)
--marker-length M             Marker side length in meters (default: 0.03)
--dictionary NAME             ArUco dictionary (default: DICT_5X5_100)
                              Choices: DICT_4X4_50, DICT_4X4_100, DICT_4X4_250,
                                       DICT_5X5_100, DICT_5X5_250, DICT_6X6_250

--samples N                   Number of samples to average (default: 10)
--min-corners N               Reject frames with fewer charuco corners (default: 8)
--no-preview                  Headless mode, auto-capture every ~0.5s
--output PATH                 Output JSON path (default: DualD455Extrinsics.json)
```

## Output schema

```json
{
    "tx": 0.5221, "ty": -0.1718, "tz": 0.3140,
    "rx": -8.61,  "ry": -54.18,  "rz": 1.11,
    "cameraASerial": "419222300508",
    "cameraBSerial": "409122300736",
    "method": "charuco_solvepnp",
    "samples": 10,
    "translationStdMm": [1.2, 0.8, 1.5],
    "rotationStdDeg": 0.18,
    "board": {
        "squaresX": 5, "squaresY": 7,
        "squareLength": 0.04, "markerLength": 0.03,
        "dictionary": "DICT_5X5_100"
    }
}
```

The first six fields (`tx … rz`) are what Unity reads. Everything else is metadata for traceability (which board, which cameras, how confident the calibration is). Translations are in **meters**, rotations are **Euler degrees** in Unity's `Quaternion.Euler(rx, ry, rz)` convention (intrinsic YXZ).
