#!/usr/bin/env python3
from __future__ import annotations

import time
import cv2
from flask import Flask, Response
from pupil_apriltags import Detector

from s1m03_capture import USBCameraCapture, gray_to_bgr

TAG_FAMILY = "tagStandard41h12"
CAM_DEVICE = "/dev/video0" # change to rpi path
W, H = 1280, 720
PORT = 8080
JPEG_QUALITY = 80

app = Flask(__name__)
cap = USBCameraCapture(device=CAM_DEVICE, width=W, height=H)

detector = Detector(
    families=TAG_FAMILY,
    nthreads=4,
    quad_decimate=2.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
)


def ensure_started():
    if cap.proc is None:
        cap.start()


def gen():
    t0 = time.time()
    frames = 0
    ensure_started()

    while True:
        ok, gray = cap.read()
        if not ok or gray is None:
            print("Read failed, restarting capture...")
            cap.release()
            ensure_started()
            continue

        frame = gray_to_bgr(gray)
        detections = detector.detect(gray, estimate_tag_pose=False)

        for det in detections:
            corners = det.corners.astype(int)
            for i in range(4):
                cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)
            c = det.center.astype(int)
            cv2.putText(
                frame,
                f"id={det.tag_id}",
                (c[0] + 5, c[1] - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

        frames += 1
        if frames % 60 == 0:
            fps = frames / (time.time() - t0)
            print(f"stream FPS ~ {fps:.1f}, tags: {[d.tag_id for d in detections]}", flush=True)

        ok_jpg, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if not ok_jpg:
            continue
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n"
        )


@app.route("/")
def index():
    return '<h3>AprilTag stream (USB Camera)</h3><img src="/stream">'


@app.route("/stream")
def stream():
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    try:
        cap.start()
        app.run(host="0.0.0.0", port=PORT, threaded=True, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        cap.release()
        print("Camera released properly.")
