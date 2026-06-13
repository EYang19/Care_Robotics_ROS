from __future__ import annotations

from typing import Optional

import cv2
import numpy as np


class USBCameraCapture:
    """Read frames from a UVC/USB camera through OpenCV + V4L2.

    This replaces the old OV9281/CSI path that used media-ctl and raw GREY frames.
    The current USB camera exposes MJPG at 1280x720, 640x400, and 640x360.
    """

    def __init__(
        self,
        device: str = "/dev/video0",
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        fourcc: str = "MJPG",
        convert_to_gray: bool = True,
    ) -> None:
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.fourcc = fourcc
        self.convert_to_gray = convert_to_gray
        self.proc: Optional[cv2.VideoCapture] = None  # compatibility with previous scripts

    def start(self) -> None:
        self.release()

        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open USB camera: {self.device}")

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Confirm actual settings. Some cameras silently choose a different mode.
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join(chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4))
        print(f"USB camera opened: {self.device} {actual_w}x{actual_h} {actual_fps:.1f}fps fourcc={fourcc_str}")

        self.proc = cap

    def read(self) -> tuple[bool, Optional[np.ndarray]]:
        if self.proc is None:
            raise RuntimeError("Capture not started")

        ok, frame = self.proc.read()
        if not ok or frame is None:
            return False, None

        if self.convert_to_gray:
            if frame.ndim == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return True, frame

    def release(self) -> None:
        if self.proc is not None:
            self.proc.release()
            self.proc = None

    def __enter__(self) -> "USBCameraCapture":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.release()


def gray_to_bgr(gray: np.ndarray) -> np.ndarray:
    if gray.ndim == 2:
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    return gray
