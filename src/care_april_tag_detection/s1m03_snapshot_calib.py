#!/usr/bin/env python3
from __future__ import annotations

import os
import time
import threading

import cv2
from flask import Flask, Response, redirect, url_for

from s1m03_capture import USBCameraCapture, gray_to_bgr

import numpy as np

CAM_DEVICE = "/dev/video0" # change to rpi 
W, H = 1280, 720
SAVE_DIR = f"output/calib_snaps_{W}x{H}"
os.makedirs(SAVE_DIR, exist_ok=True)

app = Flask(__name__)
cap = USBCameraCapture(device=CAM_DEVICE, width=W, height=H)

latest_frame = None
prev_gray = None
frame_lock = threading.Lock()
snap_count = 0
running = True


def capture_loop():
    global latest_frame, prev_gray, running

    frame_idx = 0

    while running:
        try:
            if cap.proc is None:
                print("Starting capture...")
                cap.start()

            ok, gray = cap.read()
            # print(f"loop {frame_idx}: ok={ok}, shape={None if gray is None else gray.shape}")

            if not ok or gray is None:
                print("Read failed, restarting capture...")
                cap.release()
                time.sleep(0.2)
                continue

            if prev_gray is not None:
                diff = np.mean(np.abs(gray.astype(np.int16) - prev_gray.astype(np.int16)))
                # print(f"loop {frame_idx}: frame diff = {diff:.3f}")
            else:
                print(f"loop {frame_idx}: first frame")

            prev_gray = gray.copy()

            with frame_lock:
                latest_frame = gray.copy()

            frame_idx += 1

        except Exception as e:
            print(f"Capture error: {e}")
            cap.release()
            time.sleep(0.5)


def gen():
    while True:
        with frame_lock:
            frame = None if latest_frame is None else latest_frame.copy()

        if frame is None:
            time.sleep(0.05)
            continue

        view = gray_to_bgr(frame)

        cv2.putText(
            view,
            f"USB Camera {W}x{H}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
        )

        cv2.putText(
            view,
            time.strftime("%H:%M:%S"),
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 255),
            2,
        )

        ret, jpg = cv2.imencode(".jpg", view, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ret:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Cache-Control: no-cache\r\n\r\n" + jpg.tobytes() + b"\r\n"
        )


@app.route("/")
def index():
    return f"""
    <h3>USB Camera Calibration Live View</h3>
    <p>Resolution: {W}x{H}</p>
    <a href="/snap"><button>Take Snapshot</button></a>
    <br><br>
    <img src="/stream" />
    """


@app.route("/snap")
def snap():
    global snap_count

    with frame_lock:
        frame = None if latest_frame is None else latest_frame.copy()


    if frame is not None:
        filename = os.path.join(SAVE_DIR, f"snap_{snap_count:03d}.png")
        cv2.imwrite(filename, frame)
        print(f"Saved {filename}")
        snap_count += 1

    return redirect(url_for("index"))


@app.route("/stream")
def stream():
    return Response(
        gen(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers={
            "Cache-Control": "no-cache, no-store, must-revalidate",
            "Pragma": "no-cache",
            "Expires": "0",
        },
    )


if __name__ == "__main__":
    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()

    try:
        app.run(host="0.0.0.0", port=8080, threaded=True, debug=False, use_reloader=False)
    finally:
        running = False
        cap.release()