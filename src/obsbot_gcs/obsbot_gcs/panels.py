"""Side-panel instruments: a pan/tilt position map and a joystick monitor."""

from __future__ import annotations

from PyQt5.QtCore import QPoint, QRect, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QSlider, QVBoxLayout, QWidget

from . import theme


class AttitudeMap(QWidget):
    """Top-down map of the gimbal's whole travel envelope with a position dot.

    The HUD ladders show each axis separately; this shows both at once, so it
    is obvious at a glance when the camera is parked in a corner of its range.
    """

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(170)
        self.pan = 0.0
        self.tilt = 0.0
        self.pan_limits = (-130.0, 130.0)
        self.tilt_limits = (-90.0, 90.0)
        self.trail: list[tuple[float, float]] = []

    def set_state(self, pan: float, tilt: float) -> None:
        self.pan, self.tilt = pan, tilt
        if not self.trail or abs(self.trail[-1][0] - pan) > 0.4 \
                or abs(self.trail[-1][1] - tilt) > 0.4:
            self.trail.append((pan, tilt))
            if len(self.trail) > 120:
                self.trail.pop(0)
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        box = self.rect().adjusted(10, 10, -10, -10)
        painter.setPen(QPen(theme.BORDER, 1))
        painter.setBrush(QColor(13, 17, 23))
        painter.drawRect(box)
        painter.setBrush(Qt.NoBrush)

        # Centre cross-hairs mark the home pose.
        painter.setPen(QPen(theme.BORDER, 1, Qt.DashLine))
        painter.drawLine(box.center().x(), box.y(), box.center().x(), box.bottom())
        painter.drawLine(box.x(), box.center().y(), box.right(), box.center().y())

        def to_px(pan: float, tilt: float) -> QPoint:
            pl, ph = self.pan_limits
            tl, th = self.tilt_limits
            fx = (pan - pl) / (ph - pl or 1.0)
            fy = (tilt - tl) / (th - tl or 1.0)
            # Pan is positive to the left, so mirror it to read like a map.
            return QPoint(int(box.x() + (1.0 - fx) * box.width()),
                          int(box.y() + (1.0 - fy) * box.height()))

        if len(self.trail) > 1:
            for index in range(1, len(self.trail)):
                alpha = int(150 * index / len(self.trail))
                painter.setPen(QPen(QColor(88, 166, 255, alpha), 1))
                painter.drawLine(to_px(*self.trail[index - 1]), to_px(*self.trail[index]))

        point = to_px(self.pan, self.tilt)
        painter.setPen(QPen(theme.ACCENT, 2))
        painter.setBrush(theme.ACCENT)
        painter.drawEllipse(point, 4, 4)
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(point, 9, 9)

        painter.setFont(QFont("DejaVu Sans Mono", 8))
        painter.setPen(theme.MUTED)
        painter.drawText(box.x() + 3, box.bottom() - 4, f"{self.pan_limits[1]:+.0f}")
        painter.drawText(box.right() - 32, box.bottom() - 4, f"{self.pan_limits[0]:+.0f}")
        painter.drawText(box.x() + 3, box.y() + 12, f"{self.tilt_limits[1]:+.0f}")
        painter.end()


class JoystickMonitor(QWidget):
    """Live view of the stick, so a mis-mapped axis is visible immediately."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(150)
        self.axes: list[float] = []
        self.buttons: list[int] = []
        self.pan_axis = 0
        self.tilt_axis = 1
        # Buttons that actually do something, by the number printed on the
        # stick. Everything else lights up grey and is inert.
        self.bound: dict[int, str] = {}
        self.connected = False

    def set_joy(self, axes: list[float], buttons: list[int], connected: bool) -> None:
        self.axes, self.buttons, self.connected = axes, buttons, connected
        self.update()

    def _axis(self, index: int) -> float:
        if 0 <= index < len(self.axes):
            return self.axes[index]
        return 0.0

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        size = min(self.height() - 34, 110)
        box = QRect(12, 10, size, size)
        painter.setPen(QPen(theme.BORDER, 1))
        painter.setBrush(QColor(13, 17, 23))
        painter.drawRect(box)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(theme.BORDER, 1, Qt.DashLine))
        painter.drawLine(box.center().x(), box.y(), box.center().x(), box.bottom())
        painter.drawLine(box.x(), box.center().y(), box.right(), box.center().y())

        live = theme.ACCENT if self.connected else theme.MUTED
        # joy_node follows REP-103: stick left and stick up are POSITIVE. Qt
        # screen coordinates grow right and down, so both axes must be negated
        # or the dot mirrors the operator's hand in all four directions.
        dot = QPoint(
            int(box.center().x() - self._axis(self.pan_axis) * box.width() / 2),
            int(box.center().y() - self._axis(self.tilt_axis) * box.height() / 2),
        )
        painter.setPen(QPen(live, 1))
        painter.drawLine(box.center(), dot)
        painter.setBrush(live)
        painter.drawEllipse(dot, 5, 5)
        painter.setBrush(Qt.NoBrush)

        # Only the buttons that do something. The other nine on this stick are
        # inert, so showing them would just be twelve lights to scan past.
        # Numbers are as printed on the joystick, which starts at 1.
        bx, by = box.right() + 22, box.y() + 6
        for row, (number, label) in enumerate(sorted(self.bound.items())):
            index = number - 1
            pressed = 0 <= index < len(self.buttons) and bool(self.buttons[index])
            colour = theme.ACCENT if self.connected else theme.MUTED
            cell = QRect(bx, by + row * 26, 20, 20)
            painter.setPen(QPen(colour, 1))
            painter.setBrush(colour if pressed else Qt.NoBrush)
            painter.drawRoundedRect(cell, 3, 3)
            painter.setFont(QFont("DejaVu Sans Mono", 8, QFont.Bold))
            painter.setPen(theme.BG if pressed else colour)
            painter.drawText(cell, Qt.AlignCenter, str(number))

            painter.setFont(QFont("DejaVu Sans Mono", 9))
            painter.setPen(theme.TEXT if pressed else theme.MUTED)
            painter.drawText(bx + 28, by + row * 26 + 14, label)
        painter.end()


class Readout(QWidget):
    """A big labelled number for the telemetry column."""

    def __init__(self, title: str, unit: str, parent=None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(0)

        self.title = QLabel(title)
        self.title.setObjectName("unit")
        self.value = QLabel("--")
        self.value.setObjectName("value")
        self.unit = unit

        layout.addWidget(self.title)
        layout.addWidget(self.value)

    def set(self, value: float, fmt: str = "{:+.1f}") -> None:
        self.value.setText(fmt.format(value) + self.unit)


class _JumpSlider(QSlider):
    """A slider that jumps to wherever it is clicked, not one page step."""

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton and self.orientation() == Qt.Horizontal:
            span = self.maximum() - self.minimum()
            frac = (event.pos().x() - 13) / max(1, self.width() - 26)  # handle radius
            value = self.minimum() + span * min(1.0, max(0.0, frac))
            self.setValue(int(round(value / self.singleStep())) * self.singleStep())
            event.accept()
            return
        super().mousePressEvent(event)


class SpeedBar(QWidget):
    """Slide-down bar with the joystick speed scale, 10-100 %.

    Lives on top of the video, hidden until the mouse touches the top edge of
    the window (or a speed key is pressed), and slides away again when the
    mouse leaves. Nothing is drawn while it is hidden, so an audience-facing
    screen stays clean.
    """

    changed = pyqtSignal(int)  # percent

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("speedbar")
        # A plain QWidget ignores stylesheet backgrounds unless told otherwise.
        self.setAttribute(Qt.WA_StyledBackground, True)
        self._hide_timer = QTimer(self)
        self._hide_timer.setSingleShot(True)
        self._hide_timer.timeout.connect(self.hide)

        row = QHBoxLayout(self)
        row.setContentsMargins(28, 16, 28, 16)
        row.setSpacing(22)
        row.addWidget(QLabel("STICK SPEED"))
        self.slider = _JumpSlider(Qt.Horizontal)
        self.slider.setRange(10, 100)
        self.slider.setSingleStep(10)
        self.slider.setPageStep(10)
        self.slider.setTickInterval(10)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.setValue(100)
        self.slider.valueChanged.connect(self._on_slider)
        row.addWidget(self.slider, 1)
        self.value = QLabel("100%")
        self.value.setObjectName("speedvalue")
        self.value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        row.addWidget(self.value)
        row.addWidget(QLabel("  + / −  keys"))
        self.hide()

    def percent(self) -> int:
        return self.slider.value()

    def set_percent(self, percent: int, emit: bool = True) -> None:
        percent = max(10, min(100, int(round(percent / 10.0)) * 10))
        if not emit:
            self.slider.blockSignals(True)
        self.slider.setValue(percent)
        self.value.setText(f"{percent}%")
        if not emit:
            self.slider.blockSignals(False)

    def step(self, delta: int) -> None:
        self.set_percent(self.percent() + delta)
        self.flash()

    def reveal(self) -> None:
        self._hide_timer.stop()
        self.show()
        self.raise_()

    def flash(self, ms: int = 1500) -> None:
        """Show briefly, e.g. after a keyboard change, then slide away."""
        self.reveal()
        if not self.underMouse():
            self._hide_timer.start(ms)

    def _on_slider(self, percent: int) -> None:
        self.value.setText(f"{percent}%")
        self.changed.emit(percent)

    def leaveEvent(self, event) -> None:
        self._hide_timer.start(500)
        super().leaveEvent(event)

    def enterEvent(self, event) -> None:
        self._hide_timer.stop()
        super().enterEvent(event)

