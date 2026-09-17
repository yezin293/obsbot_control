"""Video sources for the GCS.

Two ways to get pixels, because the GCS is useful in two places:

- `V4l2Source` grabs straight off the capture node. Lowest latency, no DDS
  image traffic, and it costs nothing extra -- control and capture are
  independent file descriptors, so the gimbal stays driveable while this runs.
- `TopicSource` subscribes to a sensor_msgs/Image, for when the GCS is on a
  different machine from the camera.

Both hand back the most recent frame and drop anything older; a GCS wants the
freshest picture, never a backlog.
"""

from __future__ import annotations

import os
import threading

import cv2
import numpy as np


def _release_qt_plugin_path() -> None:
    """Undo opencv-python's hijack of the Qt plugin search path.

    The pip wheel ships its own Qt and, on import, points
    QT_QPA_PLATFORM_PLUGIN_PATH at the copy inside the cv2 package. Those
    plugins are built against a different Qt than the system PyQt5, so the
    first QApplication aborts with "Could not load the Qt platform plugin
    xcb". Clearing the override lets Qt find its own plugins again.

    This has to run at import time: by the time a QApplication exists it is
    already too late. Harmless with the apt build of OpenCV, which never sets
    the variable.
    """
    path = os.environ.get("QT_QPA_PLATFORM_PLUGIN_PATH", "")
    if path and os.path.join("cv2", "qt") in path:
        del os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]


_release_qt_plugin_path()


class VideoSource:
    """Common interface: `latest()` returns a BGR frame or None."""

    def latest(self) -> np.ndarray | None:
        raise NotImplementedError

    @property
    def fps(self) -> float:
        return 0.0

    def close(self) -> None:
        pass


def _capture_spec(device: str):
    """Prefer the numeric index for a plain /dev/videoN path.

    OpenCV's V4L2 backend cannot open by name: handed a string it logs
    "backend is generally available but can't be used to capture by name",
    probes, and only then succeeds. Capture works either way, but the index
    skips the probe and the alarming-looking warning. Anything else (a
    /dev/v4l/by-id symlink, say) is passed through as a string.
    """
    import re

    match = re.fullmatch(r"/dev/video(\d+)", device)
    return int(match.group(1)) if match else device


class V4l2Source(VideoSource):
    def __init__(self, device: str, width: int = 1920, height: int = 1080,
                 fps: int = 30) -> None:
        self.device = device
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._running = True
        self._fps = 0.0
        self._error: str | None = None

        self.cap = cv2.VideoCapture(_capture_spec(device), cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open {device} for capture")
        # MJPG keeps 1080p30 within the USB budget; raw YUYV cannot sustain it.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        import time

        stamps: list[float] = []
        while self._running:
            ok, frame = self.cap.read()
            if not ok:
                self._error = "capture read failed"
                time.sleep(0.1)
                continue
            self._error = None
            with self._lock:
                self._frame = frame
            now = time.perf_counter()
            stamps.append(now)
            if len(stamps) > 30:
                stamps.pop(0)
            if len(stamps) > 1:
                self._fps = (len(stamps) - 1) / (stamps[-1] - stamps[0])

    def latest(self) -> np.ndarray | None:
        with self._lock:
            return self._frame

    @property
    def fps(self) -> float:
        return self._fps

    @property
    def error(self) -> str | None:
        return self._error

    def close(self) -> None:
        if not self._running:
            return
        self._running = False
        self._thread.join(timeout=1.0)
        self.cap.release()


class TopicSource(VideoSource):
    """Frames from a sensor_msgs/Image subscription."""

    def __init__(self, node, topic: str) -> None:
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        from rclpy.qos import QoSPresetProfiles

        self._bridge = CvBridge()
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._stamps: list[float] = []
        self._fps = 0.0
        node.create_subscription(
            Image, topic, self._on_image, QoSPresetProfiles.SENSOR_DATA.value
        )

    def _on_image(self, msg) -> None:
        import time

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        with self._lock:
            self._frame = frame
        self._stamps.append(time.perf_counter())
        if len(self._stamps) > 30:
            self._stamps.pop(0)
        if len(self._stamps) > 1:
            self._fps = (len(self._stamps) - 1) / (self._stamps[-1] - self._stamps[0])

    def latest(self) -> np.ndarray | None:
        with self._lock:
            return self._frame

    @property
    def fps(self) -> float:
        return self._fps
