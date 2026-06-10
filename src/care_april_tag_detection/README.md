# USB Camera AprilTag Detection

Python scripts for detecting AprilTags and estimating their pose using a USB camera.

The code is designed for a USB UVC camera connected via `/dev/video0`, using OpenCV and V4L2.  
It supports real-time AprilTag detection, camera calibration, and pose estimation.  
By default, this project uses the `tagStandard41h12` AprilTag family.  

## Features

- Capture images from a USB camera
- Camera calibration using checkerboard images
- Real-time AprilTag detection
- Real-time AprilTag pose estimation
- Display tag ID, distance, translation, rotation, and tilt angle

## Requirements

- Python 3
- OpenCV
- NumPy
- pupil-apriltags

Install dependencies:

```bash
pip install opencv-python numpy pupil-apriltags
```

## Files

| File | Description |
|---|---|
| `s1m03_capture.py` | Camera capture utility |
| `s1m03_snapshot_calib.py` | Capture calibration images |
| `s1m03_camera_calib.py` | Run camera calibration |
| `s1m03_realtime_detection.py` | Real-time AprilTag detection |
| `s1m03_realtime_pose.py` | Real-time AprilTag pose estimation |

## Camera Settings

The default camera device is:

```bash
/dev/video0
```

The default resolution is:

```text
1280 x 720
```

The camera input format is set to MJPG.

## Usage

### 1. Capture calibration images

```bash
python3 s1m03_snapshot_calib.py
```

Press the capture key to save checkerboard images (9 × 7).

### 2. Run camera calibration

```bash
python3 s1m03_camera_calib.py
```

The calibration result will be saved as follows in the output folder:

```text
camera_calib_1280x720.npz
```

### 3. Run real-time AprilTag detection

```bash
python3 s1m03_realtime_detection.py
```

This script detects AprilTags from the USB camera feed and displays the detected tag IDs.

### 4. Run real-time pose estimation

```bash
python3 s1m03_realtime_pose.py
```

This script estimates the position and orientation of each detected AprilTag.

Example output:

```text
id=2 z=0.153m t=(-0.094, -0.000, 0.153)m rxyz=(8.6, -4.8, -91.1)deg tilt=9.9deg
```

## Output Meaning

| Output | Meaning |
|---|---|
| `id` | AprilTag ID |
| `z` | Distance from camera to tag |
| `t` | Translation vector `(x, y, z)` in meters |
| `rxyz` | Rotation angles around x, y, and z axes |
| `tilt` | Angle between camera direction and tag normal |
| `parallel` | Whether the camera is nearly parallel to the tag |
| `centered` | Whether the tag is close to the camera center |
| `status` | Overall detection status |

## Notes

- Run calibration before pose estimation.
- Use the same resolution for calibration and real-time pose estimation.
