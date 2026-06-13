#!/usr/bin/env python3
from __future__ import annotations

import os
import time
import cv2
import numpy as np
from flask import Flask, Response
from pupil_apriltags import Detector

from s1m03_capture import USBCameraCapture, gray_to_bgr

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Int32
except ImportError:
    rclpy = None
    PoseStamped = None
    Int32 = None

# ===== Settings =====
TAG_FAMILY = "tagStandard41h12"
TAG_SIZE_M = 0.028
CALIB_PATH = "output/camera_calib_1280x720.npz" # change to your calibration file path

CAM_DEVICE = "/dev/video0" # change to rpi path 
W, H = 1280, 720
TARGET_FPS = 30
PORT = 8080

USE_UNDISTORT = True
Z_SIGN = -1

# ROS publishing. The AprilTag pose is a detected dock pose, not the robot pose.
ROS_NODE_NAME = "care_apriltag_dock_pose_publisher"
DETECTED_DOCK_POSE_TOPIC = "/detected_dock_pose"
TARGET_DOCK_TAG_TOPIC = "/target_dock_tag_id"
CAMERA_FRAME_ID = os.getenv("APRILTAG_CAMERA_FRAME", "left_docking_camera_optical_frame")
TARGET_TAG_ID = os.getenv("APRILTAG_TARGET_ID")
TARGET_TAG_ID = int(TARGET_TAG_ID) if TARGET_TAG_ID not in (None, "") else None

# Pose Evaluation (if Front-facing or not)
PARALLEL_THRESH_DEG = 10.0   # Permitable tilt of the tag plane [deg]
CENTER_THRESH_X_M = 0.03     # Permitable left/right offset from camera center [m]
CENTER_THRESH_Y_M = 0.03     # Permitable up/down offset from camera center [m]
MIN_Z_M = 0.05               # To avoid false positives when too close
MAX_Z_M = 1.50               # To avoid false positives when too far

# Detector tuning
QUAD_DECIMATE = 2.0
NTHREADS = 4
REFINE_EDGES = 0
JPEG_QUALITY = 80

# Drawing
DRAW_AXES = True
DRAW_CUBE = True
AXIS_SCALE = 0.8
CUBE_HEIGHT_SCALE = 0.6
# ====================

app = Flask(__name__)
ros_node = None
dock_pose_pub = None
target_tag_sub = None
current_target_tag_id = TARGET_TAG_ID


def load_calib(path: str):
    cal = np.load(path)
    if "K" in cal:
        K = cal["K"].astype(np.float64)
    elif "mtx" in cal:
        K = cal["mtx"].astype(np.float64)
    else:
        raise KeyError("Calibration file must contain 'K' or 'mtx'.")
    dist = cal["dist"].ravel().astype(np.float64)
    return K, dist


def project_points(camera_params, pts_cam):
    fx, fy, cx, cy = camera_params
    pts_cam = np.asarray(pts_cam, dtype=np.float64)
    z = pts_cam[:, 2:3]
    z_safe = np.where(z <= 1e-9, 1e-9, z)
    x = pts_cam[:, 0:1] / z_safe
    y = pts_cam[:, 1:2] / z_safe
    u = fx * x + cx
    v = fy * y + cy
    return np.hstack([u, v])


def draw_axes(img, camera_params, R, t, axis_length, z_sign=1):
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    t = np.asarray(t, dtype=np.float64).reshape(3, 1)

    pts_tag = np.array([
        [0.0, 0.0, 0.0],
        [axis_length, 0.0, 0.0],
        [0.0, axis_length, 0.0],
        [0.0, 0.0, float(z_sign) * axis_length],
    ], dtype=np.float64)

    pts_cam = (R @ pts_tag.T + t).T
    if np.any(pts_cam[:, 2] <= 1e-6):
        return

    pts_img = project_points(camera_params, pts_cam).astype(int)
    o = tuple(pts_img[0]); x = tuple(pts_img[1]); y = tuple(pts_img[2]); z = tuple(pts_img[3])
    cv2.arrowedLine(img, o, x, (0, 0, 255), 3, tipLength=0.2)   # X red
    cv2.arrowedLine(img, o, y, (0, 255, 0), 3, tipLength=0.2)   # Y green
    cv2.arrowedLine(img, o, z, (255, 0, 0), 3, tipLength=0.2)   # Z blue


def draw_cube(img, camera_params, R, t, tag_size, height, z_sign):
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    t = np.asarray(t, dtype=np.float64).reshape(3, 1)

    s = tag_size / 2.0
    zh = float(z_sign) * height

    bottom = np.array([[-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0]], dtype=np.float64)
    top = bottom.copy(); top[:, 2] = zh

    pts_tag = np.vstack([bottom, top])
    pts_cam = (R @ pts_tag.T + t).T
    if np.any(pts_cam[:, 2] <= 1e-6):
        return

    pts_img = project_points(camera_params, pts_cam).astype(int)
    b = pts_img[0:4]; tp = pts_img[4:8]

    for i in range(4):
        cv2.line(img, tuple(b[i]), tuple(b[(i+1) % 4]), (0, 255, 0), 2)
        cv2.line(img, tuple(tp[i]), tuple(tp[(i+1) % 4]), (255, 0, 0), 2)
        cv2.line(img, tuple(b[i]), tuple(tp[i]), (0, 255, 255), 2)


def tilt_deg_from_R(R):
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    n_cam = R @ np.array([0.0, 0.0, 1.0])
    cos_theta = np.clip(abs(float(n_cam[2])), -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_theta)))


def rotation_matrix_to_euler_xyz_deg(R: np.ndarray) -> tuple[float, float, float]:
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0.0

    return tuple(np.degrees([x, y, z]))


def rotation_matrix_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(R))

    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def init_ros_publisher() -> None:
    global ros_node, dock_pose_pub, target_tag_sub

    if rclpy is None:
        print("ROS publishing disabled: rclpy/geometry_msgs not available", flush=True)
        return
    if ros_node is not None:
        return

    rclpy.init(args=None)
    ros_node = rclpy.create_node(ROS_NODE_NAME)
    dock_pose_pub = ros_node.create_publisher(PoseStamped, DETECTED_DOCK_POSE_TOPIC, 10)
    target_tag_sub = ros_node.create_subscription(
        Int32,
        TARGET_DOCK_TAG_TOPIC,
        on_target_tag_id,
        10,
    )
    ros_node.get_logger().info(
        f"Publishing AprilTag dock poses on {DETECTED_DOCK_POSE_TOPIC} "
        f"in frame {CAMERA_FRAME_ID}"
    )
    ros_node.get_logger().info(
        f"Listening for target dock tag IDs on {TARGET_DOCK_TAG_TOPIC}"
    )


def on_target_tag_id(msg: Int32) -> None:
    global current_target_tag_id

    current_target_tag_id = int(msg.data) if msg.data >= 0 else None
    if ros_node is not None:
        ros_node.get_logger().info(f"Target AprilTag ID set to {current_target_tag_id}")


def spin_ros_once() -> None:
    if ros_node is not None:
        rclpy.spin_once(ros_node, timeout_sec=0.0)


def publish_detected_dock_pose(tag_id: int, R: np.ndarray, t: np.ndarray) -> None:
    if ros_node is None or dock_pose_pub is None:
        return
    if current_target_tag_id is not None and tag_id != current_target_tag_id:
        return

    tx, ty, tz = [float(v) for v in np.asarray(t).reshape(-1)]
    qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

    msg = PoseStamped()
    msg.header.stamp = ros_node.get_clock().now().to_msg()
    msg.header.frame_id = CAMERA_FRAME_ID
    msg.pose.position.x = tx
    msg.pose.position.y = ty
    msg.pose.position.z = tz
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw

    dock_pose_pub.publish(msg)


def classify_tag(t: np.ndarray, tilt_deg: float) -> tuple[str, tuple[int, int, int], bool, bool]:
    tx, ty, tz = [float(v) for v in t]

    is_parallel = tilt_deg <= PARALLEL_THRESH_DEG
    is_centered = abs(tx) <= CENTER_THRESH_X_M and abs(ty) <= CENTER_THRESH_Y_M
    is_in_range = MIN_Z_M <= tz <= MAX_Z_M
    is_front = is_parallel and is_centered and is_in_range

    if is_front:
        return "FRONT_OK", (0, 255, 0), is_parallel, is_centered
    if is_parallel and not is_centered:
        return "PARALLEL_NOT_FRONT", (0, 255, 255), is_parallel, is_centered
    return "TILTED", (0, 165, 255), is_parallel, is_centered


K, dist = load_calib(CALIB_PATH)
cap = USBCameraCapture(device=CAM_DEVICE, width=W, height=H)

if USE_UNDISTORT:
    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (W, H), 0)
    map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, newK, (W, H), cv2.CV_16SC2)
    fx, fy, cx, cy = float(newK[0, 0]), float(newK[1, 1]), float(newK[0, 2]), float(newK[1, 2])
else:
    map1 = map2 = None
    fx, fy, cx, cy = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])

camera_params = (fx, fy, cx, cy)

detector = Detector(
    families=TAG_FAMILY,
    nthreads=NTHREADS,
    quad_decimate=QUAD_DECIMATE,
    quad_sigma=0.0,
    refine_edges=REFINE_EDGES,
    decode_sharpening=0.25,
)

axis_length = TAG_SIZE_M * AXIS_SCALE
cube_h = TAG_SIZE_M * CUBE_HEIGHT_SCALE


def ensure_started():
    if cap.proc is None:
        cap.start()
    init_ros_publisher()


def gen():
    t0 = time.time()
    frames = 0
    last_log = 0.0
    ensure_started()

    while True:
        ok, gray = cap.read()
        if not ok or gray is None:
            print("Read failed, restarting capture...")
            cap.release()
            ensure_started()
            continue

        vis_gray = cv2.remap(gray, map1, map2, cv2.INTER_LINEAR) if USE_UNDISTORT else gray
        vis = gray_to_bgr(vis_gray)

        dets = detector.detect(
            vis_gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=TAG_SIZE_M,
        )
        spin_ros_once()

        for d in dets:
            corners = d.corners.astype(int)
            center = d.center.astype(int)
            t = np.asarray(d.pose_t).reshape(-1)
            rx, ry, rz = rotation_matrix_to_euler_xyz_deg(d.pose_R)
            tilt = tilt_deg_from_R(d.pose_R)
            status, color, is_parallel, is_centered = classify_tag(t, tilt)
            publish_detected_dock_pose(d.tag_id, d.pose_R, d.pose_t)

            for i in range(4):
                cv2.line(vis, tuple(corners[i]), tuple(corners[(i + 1) % 4]), color, 2)

            if DRAW_AXES:
                draw_axes(vis, camera_params, d.pose_R, d.pose_t, axis_length, z_sign=Z_SIGN)
            if DRAW_CUBE:
                draw_cube(vis, camera_params, d.pose_R, d.pose_t, TAG_SIZE_M, cube_h, z_sign=Z_SIGN)

            tx, ty, tz = [float(v) for v in t]
            cv2.putText(vis, f"id={d.tag_id} {status}", (center[0] + 10, center[1] - 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(vis, f"z={tz:.3f}m  tx={tx:.3f} ty={ty:.3f}", (center[0] + 10, center[1] - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
            cv2.putText(vis, f"rxyz=({rx:.1f},{ry:.1f},{rz:.1f})  tilt={tilt:.1f}", (center[0] + 10, center[1] + 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            if is_centered:
                cv2.putText(vis, "CENTERED", (center[0] + 10, center[1] + 48),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            else:
                cv2.putText(vis, "OFF-CENTER", (center[0] + 10, center[1] + 48),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            now = time.time()
            if now - last_log > 1.0:
                print(
                    f"id={d.tag_id} z={tz:.3f}m t=({tx:.3f}, {ty:.3f}, {tz:.3f})m "
                    f"rxyz=({rx:.1f}, {ry:.1f}, {rz:.1f})deg tilt={tilt:.1f}deg ",
                    flush=True,
                )
                # Show whether the pose is parallel and/or centered if needed
                # print(
                #     f"parallel={is_parallel} centered={is_centered} status={status}",
                #     flush=True,
                # )
                last_log = now

        # camera optical center guide
        cv2.drawMarker(vis, (int(cx), int(cy)), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
        cv2.putText(vis, f"front rule: tilt<={PARALLEL_THRESH_DEG}deg, |tx|<={CENTER_THRESH_X_M:.2f}m, |ty|<={CENTER_THRESH_Y_M:.2f}m",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        frames += 1
        # if frames % 120 == 0:
        #     fps = frames / (time.time() - t0)
        #     print(f"FPS~{fps:.1f} tags={[d.tag_id for d in dets]}", flush=True)

        ok, jpg = cv2.imencode('.jpg', vis, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if not ok:
            continue
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n")


@app.route('/')
def index():
    return f"""
    <h3>USB Camera AprilTag Pose + Front Detection</h3>
    <p>Green=FRONT_OK, Yellow=PARALLEL_NOT_FRONT, Orange=TILTED</p>
    <p>front rule: tilt&lt;={PARALLEL_THRESH_DEG}deg, |tx|&lt;={CENTER_THRESH_X_M:.2f}m, |ty|&lt;={CENTER_THRESH_Y_M:.2f}m</p>
    <img src="/stream" />
    """


@app.route('/stream')
def stream():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    try:
        cap.start()
        init_ros_publisher()
        app.run(host='0.0.0.0', port=PORT, threaded=True, debug=False, use_reloader=False)
    finally:
        cap.release()
        if ros_node is not None:
            ros_node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
