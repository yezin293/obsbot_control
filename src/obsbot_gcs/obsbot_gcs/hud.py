"""Video widget with a PTZ head-up display.

The overlay answers the three questions a camera operator actually has while
flying the gimbal: where am I pointed, how much travel is left before I hit an
end stop, and is my input reaching the camera.
"""

from __future__ import annotations

import math
import time

import numpy as np
from PyQt5.QtCore import QPoint, QRect, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import QWidget

from . import theme

_HAS_BGR888 = hasattr(QImage, "Format_BGR888")


class VideoHud(QWidget):
    """Letterboxed live view plus PTZ overlay. Click to point the camera."""

    clicked = pyqtSignal(float, float)  # normalised offset from centre, -0.5..0.5
    zoomed = pyqtSignal(float)          # wheel notches, + = zoom in

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(640, 400)
        self.setCursor(Qt.CrossCursor)

        self.frame: np.ndarray | None = None
        self._buffer: np.ndarray | None = None
        self.fps = 0.0

        self.pan = 0.0
        self.tilt = 0.0
        self.zoom = 0.0
        self.pan_limits = (-130.0, 130.0)
        self.tilt_limits = (-90.0, 90.0)

        self.link = False
        self.joy = False
        # Off means nothing at all is drawn over the picture. That is the point
        # for a screen an audience looks at; the sidebar still has the numbers.
        self.show_overlay = False
        # Just the two travel bars, no text block: enough to see how far from
        # centre the camera is and when it is about to hit an end stop, for a
        # display with no sidebar (view.launch.py).
        self.show_ladders = False
        self.status_text = "waiting for driver"

        self._click_marker: tuple[float, float, float] | None = None
        self._video_rect = QRect()

    # -- state feed ----------------------------------------------------------

    def set_frame(self, frame: np.ndarray | None, fps: float) -> None:
        self.frame = frame
        self.fps = fps

    def set_state(self, pan: float, tilt: float, zoom: float) -> None:
        self.pan, self.tilt, self.zoom = pan, tilt, zoom

    # -- input ---------------------------------------------------------------

    def mousePressEvent(self, event) -> None:
        if event.button() != Qt.LeftButton or self._video_rect.isEmpty():
            return
        rect = self._video_rect
        if not rect.contains(event.pos()):
            return
        u = (event.x() - rect.x()) / rect.width() - 0.5
        v = (event.y() - rect.y()) / rect.height() - 0.5
        self._click_marker = (event.x(), event.y(), time.perf_counter())
        self.clicked.emit(u, v)

    def wheelEvent(self, event) -> None:
        # angleDelta is in eighths of a degree; 120 units is one notch on a
        # normal mouse, while a touchpad sends a stream of small deltas.
        notches = event.angleDelta().y() / 120.0
        if notches:
            self.zoomed.emit(notches)
            event.accept()

    # -- painting ------------------------------------------------------------

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), theme.BG)

        self._video_rect = self._draw_video(painter)
        if self.show_overlay or self.show_ladders:
            self._draw_pan_ladder(painter, self._video_rect)
            self._draw_tilt_ladder(painter, self._video_rect)
        if self.show_overlay:
            self._draw_crosshair(painter, self._video_rect)
            self._draw_telemetry(painter, self._video_rect)
        # The click ripple stays either way: it is transient feedback that the
        # click landed, not a readout, and without it click-to-point feels dead.
        self._draw_click_marker(painter)
        painter.end()

    def _draw_video(self, painter: QPainter) -> QRect:
        if self.frame is None:
            painter.setPen(theme.MUTED)
            painter.setFont(QFont("DejaVu Sans Mono", 12))
            painter.drawText(self.rect(), Qt.AlignCenter, f"NO VIDEO\n{self.status_text}")
            # Still give the overlay a sane canvas so the ladders stay visible.
            return self.rect().adjusted(8, 8, -8, -8)

        # QImage does not copy, so the buffer must outlive the paint.
        # Format_BGR888 arrived in Qt 5.14; on older Qt (Ubuntu 20.04 ships
        # 5.12) swap the channels once and hand it RGB888 instead.
        if _HAS_BGR888:
            self._buffer = np.ascontiguousarray(self.frame)
            fmt = QImage.Format_BGR888
        else:
            self._buffer = np.ascontiguousarray(self.frame[..., ::-1])
            fmt = QImage.Format_RGB888
        h, w, _ = self._buffer.shape
        image = QImage(self._buffer.data, w, h, 3 * w, fmt)

        scale = min(self.width() / w, self.height() / h)
        vw, vh = int(w * scale), int(h * scale)
        rect = QRect((self.width() - vw) // 2, (self.height() - vh) // 2, vw, vh)
        painter.drawPixmap(rect, QPixmap.fromImage(image))
        return rect

    def _draw_crosshair(self, painter: QPainter, rect: QRect) -> None:
        cx, cy = rect.center().x(), rect.center().y()
        painter.setPen(QPen(theme.HUD, 1))
        for dx in (-1, 1):
            painter.drawLine(cx + dx * 8, cy, cx + dx * 26, cy)
            painter.drawLine(cx, cy + dx * 8, cx, cy + dx * 26)
        painter.drawEllipse(QPoint(cx, cy), 3, 3)

    def _scale(self, rect: QRect) -> float:
        """Overlay scale factor: 1.0 at 720p, larger on a big display."""
        return max(1.0, rect.height() / 720.0)

    def _draw_pan_ladder(self, painter: QPainter, rect: QRect) -> None:
        """Horizontal travel bar along the bottom edge.

        Pan is positive to the left (REP-103), so the axis is mirrored: the
        marker slides left when the camera looks left. Matching the operator's
        view matters more here than matching the sign convention.
        """
        low, high = self.pan_limits
        k = self._scale(rect)
        margin, height = int(60 * k), int(10 * k)
        bar = QRect(rect.x() + margin, rect.bottom() - int(34 * k),
                    rect.width() - 2 * margin, height)
        self._draw_bar(painter, bar, low, high, self.pan, "PAN",
                       horizontal=True, mirror=True, k=k)

    def _draw_tilt_ladder(self, painter: QPainter, rect: QRect) -> None:
        """Vertical travel bar along the right edge, up at the top."""
        low, high = self.tilt_limits
        k = self._scale(rect)
        margin, width = int(60 * k), int(10 * k)
        bar = QRect(rect.right() - int(34 * k), rect.y() + margin,
                    width, rect.height() - 2 * margin)
        self._draw_bar(painter, bar, low, high, self.tilt, "TILT",
                       horizontal=False, mirror=True, k=k)

    def _draw_bar(self, painter: QPainter, bar: QRect, low: float, high: float,
                  value: float, label: str, horizontal: bool,
                  mirror: bool, k: float = 1.0) -> None:
        painter.setPen(QPen(theme.HUD_DIM, 1))
        painter.setBrush(QColor(13, 17, 23, 140))
        painter.drawRect(bar)
        painter.setBrush(Qt.NoBrush)

        span = high - low or 1.0
        frac = (value - low) / span
        # Warn as the axis approaches an end stop: the last 8% of travel.
        near_limit = frac < 0.08 or frac > 0.92
        colour = theme.WARN if near_limit else theme.HUD
        along = (1.0 - frac) if mirror else frac
        t = max(1, int(3 * k))   # marker half-width / tick overhang

        # Centre tick marks the home pose.
        painter.setPen(QPen(theme.HUD_DIM, 1))
        if horizontal:
            mid = bar.x() + bar.width() // 2
            painter.drawLine(mid, bar.y() - t, mid, bar.bottom() + t)
            pos = int(bar.x() + along * bar.width())
            marker = QRect(pos - t, bar.y() - t - 1, 2 * t, bar.height() + 2 * t + 2)
        else:
            mid = bar.y() + bar.height() // 2
            painter.drawLine(bar.x() - t, mid, bar.right() + t, mid)
            pos = int(bar.y() + along * bar.height())
            marker = QRect(bar.x() - t - 1, pos - t, bar.width() + 2 * t + 2, 2 * t)

        painter.setPen(QPen(colour, 1))
        painter.setBrush(colour)
        painter.drawRect(marker)
        painter.setBrush(Qt.NoBrush)

        # Current angle rides with the marker, so the number is where the eye
        # already is. Kept on the picture side of the bar.
        painter.setFont(QFont("DejaVu Sans Mono", int(10 * k), QFont.Bold))
        painter.setPen(colour)
        reading = f"{value:+.0f}°"
        if horizontal:
            painter.drawText(QRect(pos - int(40 * k), bar.y() - int(24 * k), int(80 * k), int(20 * k)),
                             Qt.AlignHCenter | Qt.AlignBottom, reading)
        else:
            painter.drawText(QRect(bar.x() - int(70 * k), pos - int(10 * k), int(62 * k), int(20 * k)),
                             Qt.AlignRight | Qt.AlignVCenter, reading)

        # End labels follow the mirroring, so they always name the end the
        # marker actually travels toward.
        near_end, far_end = (high, low) if mirror else (low, high)
        painter.setFont(QFont("DejaVu Sans Mono", int(8 * k)))
        painter.setPen(theme.HUD_DIM)
        if horizontal:
            painter.drawText(bar.x(), bar.y() - int(6 * k), f"{label} {near_end:+.0f}")
            painter.drawText(bar.right() - int(26 * k), bar.y() - int(6 * k), f"{far_end:+.0f}")
        else:
            painter.drawText(bar.right() - int(68 * k), bar.y() - int(6 * k),
                             f"{label} {near_end:+.0f}")
            painter.drawText(bar.right() - int(30 * k), bar.bottom() + int(14 * k), f"{far_end:+.0f}")

    def _draw_telemetry(self, painter: QPainter, rect: QRect) -> None:
        painter.setFont(QFont("DejaVu Sans Mono", 11, QFont.Bold))
        x, y = rect.x() + 14, rect.y() + 26

        def line(text: str, colour: QColor, dy: int = 18) -> None:
            nonlocal y
            painter.setPen(colour)
            painter.drawText(x, y, text)
            y += dy

        line(f"PAN  {self.pan:+7.1f}°", theme.HUD)
        line(f"TILT {self.tilt:+7.1f}°", theme.HUD)
        line(f"ZOOM {self.zoom * 100:6.0f}%", theme.HUD)

        # Status chips, top right.
        painter.setFont(QFont("DejaVu Sans Mono", 9, QFont.Bold))
        chips = [
            ("LINK", theme.OK if self.link else theme.DANGER),
            ("JOY", theme.OK if self.joy else theme.MUTED),
            (f"{self.fps:4.1f}FPS", theme.HUD_DIM),
        ]
        cx = rect.right() - 52
        cy = rect.y() + 16
        for text, colour in reversed(chips):
            width = 8 * len(text) + 12
            cx -= width + 6
            chip = QRect(cx, cy, width, 20)
            painter.setPen(QPen(colour, 1))
            painter.setBrush(QColor(13, 17, 23, 170))
            painter.drawRoundedRect(chip, 3, 3)
            painter.setBrush(Qt.NoBrush)
            painter.drawText(chip, Qt.AlignCenter, text)

    def _draw_click_marker(self, painter: QPainter) -> None:
        if self._click_marker is None:
            return
        mx, my, stamp = self._click_marker
        age = time.perf_counter() - stamp
        if age > 0.6:
            self._click_marker = None
            return
        # Expanding, fading ring: confirms the click landed and where.
        alpha = int(255 * (1.0 - age / 0.6))
        radius = 6 + 26 * (age / 0.6)
        painter.setPen(QPen(QColor(88, 166, 255, alpha), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPoint(int(mx), int(my)), int(radius), int(radius))


def pixel_to_angles(u: float, v: float, zoom: float, hfov_deg: float,
                    aspect: float, zoom_max: float) -> tuple[float, float]:
    """Convert a normalised click offset into a pan/tilt delta in degrees.

    Digital zoom narrows the effective field of view, so the same pixel offset
    means a smaller angle when zoomed in -- without this the camera overshoots
    every click at high zoom.
    """
    factor = 1.0 + zoom * (zoom_max - 1.0)
    hfov = 2.0 * math.degrees(math.atan(math.tan(math.radians(hfov_deg) / 2.0) / factor))
    vfov = 2.0 * math.degrees(math.atan(math.tan(math.radians(hfov) / 2.0) / aspect))
    # Image x grows to the right, but pan is positive to the left (REP-103);
    # image y grows downward, tilt is positive up. Both therefore negate.
    return -u * hfov, -v * vfov
