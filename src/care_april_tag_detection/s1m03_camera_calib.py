#!/usr/bin/env python3
from __future__ import annotations

import glob
import os

import cv2
import numpy as np

CHECKERBOARD = (9, 7)          # inner corners (columns, rows)
SQUARE_SIZE = 0.02             # meters
IMG_GLOB = "output/calib_snaps_1280x720/*.png"
OUT_FILE = "output/camera_calib_1280x720.npz"

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints = []

images = sorted(glob.glob(IMG_GLOB))
if len(images) == 0:
    raise RuntimeError(f"No images found: {IMG_GLOB}")

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
used = 0
img_size = None

for fname in images:
    img = cv2.imread(fname, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"[SKIP] Could not read: {fname}")
        continue

    img_size = img.shape[::-1]
    ret, corners = cv2.findChessboardCornersSB(img, CHECKERBOARD, None)

    if not ret:
        ret2, corners2 = cv2.findChessboardCorners(
            img,
            CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if not ret2:
            print(f"[MISS] {os.path.basename(fname)}")
            continue
        corners = cv2.cornerSubPix(img, corners2, (11, 11), (-1, -1), criteria)

    objpoints.append(objp)
    imgpoints.append(corners)
    used += 1
    print(f"[OK] {os.path.basename(fname)}")

if used < 10:
    print(f"Warning: only {used} usable images. Aim for 15-30 for better accuracy.")

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)

total_error = 0.0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    total_error += error
mean_error = total_error / len(objpoints)

print("\n==== Calibration Result ====")
print("Used images:", used)
print("Image size:", img_size)
print("Camera matrix (K):\n", mtx)
print("Distortion:\n", dist.ravel())
print(f"Mean reprojection error (px): {mean_error:.4f}")

np.savez(OUT_FILE, K=mtx, mtx=mtx, dist=dist, img_size=np.array(img_size), reproj_err=mean_error)
print("Saved to:", OUT_FILE)
