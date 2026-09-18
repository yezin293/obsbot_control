"""OBSBOT ground control station.

One window with the live view, where the gimbal is pointed, what the joystick
is doing, and the controls to move it. Runs alongside the joystick teleop
stack -- it does not take the stick's place, it shows you what the stick is
doing and gives you click-to-point and presets on top.

Threading: rclpy spins on its own thread, Qt owns the main thread, and the two
meet only through the bridge node's plain attributes, polled by a Qt timer.
No ROS callback ever touches a widget.
"""

from __future__ import annotations

import json
import math
import os
import signal
import sys
import threading
import time

# Imported ahead of PyQt5 on purpose: this pulls in OpenCV, and the pip build
# of OpenCV redirects Qt's plugin search path to its own bundled copy on
# import. `video` undoes that, and it has to happen before any Qt plugin is
# resolved. See _release_qt_plugin_path().
from .video import TopicSource, V4l2Source  # noqa: I001  isort:skip

import rclpy
from geometry_msgs.msg import Vector3
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QShortcut,
    QVBoxLayout,
    QWidget,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState, Joy
from std_srvs.srv import Trigger

from . import theme
from .hud import VideoHud, pixel_to_angles
from .panels import AttitudeMap, JoystickMonitor, Readout

PRESET_PATH = os.path.expanduser("~/.config/obsbot_gcs/presets.json")


class GcsBridge(Node):
    """ROS side of the GCS: collects telemetry, sends pose commands."""

    def __init__(self) -> None:
        super().__init__("obsbot_gcs")

        self.declare_parameter("video_device", "")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("video_width", 1920)
        self.declare_parameter("video_height", 1080)
        self.declare_parameter("video_fps", 30)

        # Presentation. The defaults keep the picture clean -- nothing is drawn
        # over the video -- because this ends up on a screen an audience sees.
        self.declare_parameter("show_overlay", False)
        self.declare_parameter("show_ladders", False)
        self.declare_parameter("show_panels", True)
        self.declare_parameter("fullscreen", False)

        # Field of view at full wide, used to turn a click into an angle.
        self.declare_parameter("hfov_deg", 78.0)
        self.declare_parameter("zoom_max_factor", 4.0)

        self.declare_parameter("pan_limit", 130.0)
        self.declare_parameter("tilt_limit", 90.0)
        self.declare_parameter("jog_step_deg", 3.0)
        self.declare_parameter("zoom_wheel_step", 0.05)  # per mouse-wheel notch

        # Mirrors of the joy_to_ptz mapping, for the monitor panel only.
        self.declare_parameter("pan_axis", 0)
        self.declare_parameter("tilt_axis", 1)
        self.declare_parameter("zoom_in_button", 1)
        self.declare_parameter("home_button", 2)
        self.declare_parameter("zoom_out_button", 3)

        self.pan = 0.0
        self.tilt = 0.0
        self.zoom = 0.0
        self.pan_rate = 0.0
        self.tilt_rate = 0.0
        self.state_time = 0.0
        self.joy_axes: list[float] = []
        self.joy_buttons: list[int] = []
        self.joy_time = 0.0

        self.create_subscription(JointState, "ptz_state", self._on_state, 10)
        self.create_subscription(
            Joy, "joy", self._on_joy, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.goto_pub = self.create_publisher(Vector3, "goto_ptz", 10)
        self.home_client = self.create_client(Trigger, "home")
        self.release_client = self.create_client(Trigger, "release_stream")

    # -- telemetry -----------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name == "pan":
                self.pan = math.degrees(position)
            elif name == "tilt":
                self.tilt = math.degrees(position)
            elif name == "zoom":
                self.zoom = position
        for name, velocity in zip(msg.name, msg.velocity):
            if name == "pan":
                self.pan_rate = math.degrees(velocity)
            elif name == "tilt":
                self.tilt_rate = math.degrees(velocity)
        self.state_time = self._now()

    def speed_dps(self) -> float:
        """Current commanded gimbal speed, for the operator's readout."""
        return max(abs(self.pan_rate), abs(self.tilt_rate))

    def _on_joy(self, msg: Joy) -> None:
        self.joy_axes = list(msg.axes)
        self.joy_buttons = list(msg.buttons)
        self.joy_time = self._now()

    def link_ok(self) -> bool:
        return self.state_time > 0.0 and (self._now() - self.state_time) < 1.0

    def joy_ok(self) -> bool:
        return self.joy_time > 0.0 and (self._now() - self.joy_time) < 2.0

    def bound_buttons(self) -> dict[int, str]:
        """Joystick buttons that do something, by printed number."""
        bindings = {
            "zoom_in_button": "zoom in",
            "zoom_out_button": "zoom out",
            "home_button": "home",
        }
        out: dict[int, str] = {}
        for param, label in bindings.items():
            number = int(self.get_parameter(param).value)
            if number >= 1:
                out[number] = label
        return out

    # -- commands ------------------------------------------------------------

    def goto(self, pan_deg: float, tilt_deg: float, zoom: float | None = None) -> None:
        pan_limit = float(self.get_parameter("pan_limit").value)
        tilt_limit = float(self.get_parameter("tilt_limit").value)
        msg = Vector3()
        msg.x = math.radians(max(-pan_limit, min(pan_limit, pan_deg)))
        msg.y = math.radians(max(-tilt_limit, min(tilt_limit, tilt_deg)))
        msg.z = float("nan") if zoom is None else max(0.0, min(1.0, zoom))
        self.goto_pub.publish(msg)

    def set_zoom(self, zoom: float) -> None:
        """Zoom without disturbing pan/tilt: NaN leaves those axes alone."""
        zoom = max(0.0, min(1.0, zoom))
        msg = Vector3()
        msg.x = float("nan")
        msg.y = float("nan")
        msg.z = zoom
        self.goto_pub.publish(msg)
        # Apply locally too. ptz_state only arrives at 20 Hz, so a fast scroll
        # would otherwise keep reading a stale value and stop accumulating.
        self.zoom = zoom

    def release_driver_stream(self, timeout: float = 1.5) -> bool:
        """Ask the driver to drop its keepalive stream so we can capture.

        Velocity commands only work while the camera streams, so the driver
        keeps a stream of its own whenever nobody else does -- which is exactly
        what blocks our capture from opening. Only one process can stream.
        Called from the GUI thread; the executor thread answers the future.
        """
        if not self.release_client.wait_for_service(timeout_sec=timeout):
            return False
        future = self.release_client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.done() and future.result() is not None and future.result().success

    def home(self) -> None:
        if self.home_client.service_is_ready():
            self.home_client.call_async(Trigger.Request())
        else:
            self.get_logger().warning("home service unavailable")


class GcsWindow(QMainWindow):
    def __init__(self, bridge: GcsBridge) -> None:
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle("OBSBOT GCS")
        self.setStyleSheet(theme.STYLESHEET)

        self.presets: dict[str, list[float]] = self._load_presets()
        self.store_mode = False

        self.hud = VideoHud()
        self.hud.clicked.connect(self._on_hud_click)
        self.hud.zoomed.connect(self._on_hud_wheel)
        self.hud.show_overlay = bool(bridge.get_parameter("show_overlay").value)
        self.hud.show_ladders = bool(bridge.get_parameter("show_ladders").value)
        pan_limit = float(bridge.get_parameter("pan_limit").value)
        tilt_limit = float(bridge.get_parameter("tilt_limit").value)
        self.hud.pan_limits = (-pan_limit, pan_limit)
        self.hud.tilt_limits = (-tilt_limit, tilt_limit)

        self.show_panels = bool(bridge.get_parameter("show_panels").value)
        root = QWidget()
        layout = QHBoxLayout(root)
        # With the panels hidden the video goes edge to edge; a margin would
        # just be a grey frame on a stadium screen.
        margin = 8 if self.show_panels else 0
        layout.setContentsMargins(margin, margin, margin, margin)
        layout.setSpacing(margin)

        # Always built, so every refresh and shortcut path stays valid; only
        # added to the layout when panels are on. An unparented widget that is
        # never shown costs nothing.
        self.controls = self._build_controls()
        self.sidebar = self._build_sidebar()

        left = QVBoxLayout()
        left.setSpacing(margin)
        left.addWidget(self.hud, 1)
        if self.show_panels:
            left.addWidget(self.controls)
        layout.addLayout(left, 1)
        if self.show_panels:
            layout.addWidget(self.sidebar)
        self.setCentralWidget(root)

        self.video = self._open_video()
        self._bind_shortcuts()

        timer = QTimer(self)
        timer.timeout.connect(self._refresh)
        timer.start(33)

    # -- construction --------------------------------------------------------

    def _open_video(self):
        topic = str(self.bridge.get_parameter("image_topic").value)
        if topic:
            self.hud.status_text = f"subscribing to {topic}"
            return TopicSource(self.bridge, topic)

        device = str(self.bridge.get_parameter("video_device").value)
        if not device:
            from obsbot_ptz.v4l2_ptz import find_obsbot, PtzError

            try:
                device = find_obsbot()
            except PtzError as exc:
                self.hud.status_text = str(exc)
                return None
        size = (
            int(self.bridge.get_parameter("video_width").value),
            int(self.bridge.get_parameter("video_height").value),
            int(self.bridge.get_parameter("video_fps").value),
        )
        try:
            return V4l2Source(device, *size)
        except RuntimeError as exc:
            first_error = exc

        # Most likely the driver's keepalive stream holds the device. Ask it
        # to step aside, then retry while it winds down.
        if self.bridge.release_driver_stream():
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                time.sleep(0.25)
                try:
                    return V4l2Source(device, *size)
                except RuntimeError as exc:
                    first_error = exc
        self.hud.status_text = str(first_error)
        return None

    def _build_sidebar(self) -> QWidget:
        panel = QWidget()
        panel.setFixedWidth(300)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        telemetry = QGroupBox("TELEMETRY")
        grid = QGridLayout(telemetry)
        self.pan_out = Readout("PAN", "°")
        self.tilt_out = Readout("TILT", "°")
        self.zoom_out = Readout("ZOOM", "%")
        self.rate_out = Readout("VIDEO", "fps")
        grid.addWidget(self.pan_out, 0, 0)
        grid.addWidget(self.tilt_out, 0, 1)
        grid.addWidget(self.zoom_out, 1, 0)
        grid.addWidget(self.rate_out, 1, 1)
        layout.addWidget(telemetry)

        position = QGroupBox("POSITION")
        box = QVBoxLayout(position)
        self.map = AttitudeMap()
        self.map.pan_limits = self.hud.pan_limits
        self.map.tilt_limits = self.hud.tilt_limits
        box.addWidget(self.map)
        layout.addWidget(position)

        joystick = QGroupBox("JOYSTICK")
        box = QVBoxLayout(joystick)
        self.joy_monitor = JoystickMonitor()
        self.joy_monitor.pan_axis = int(self.bridge.get_parameter("pan_axis").value)
        self.joy_monitor.tilt_axis = int(self.bridge.get_parameter("tilt_axis").value)
        self.joy_monitor.bound = self.bridge.bound_buttons()
        box.addWidget(self.joy_monitor)
        self.joy_status = QLabel("no joystick")
        self.joy_status.setObjectName("unit")
        box.addWidget(self.joy_status)
        layout.addWidget(joystick)

        layout.addStretch(1)
        hint = QLabel(
            "click video · point there\n"
            "wheel · zoom    arrows · jog\n"
            "H · home        1-4 · preset\n"
            "S · store"
        )
        hint.setObjectName("unit")
        layout.addWidget(hint)
        return panel

    def _build_controls(self) -> QWidget:
        bar = QWidget()
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(0, 0, 0, 0)

        home = QPushButton("HOME")
        home.clicked.connect(self.bridge.home)
        layout.addWidget(home)

        centre = QPushButton("RE-CENTRE VIEW")
        centre.clicked.connect(lambda: self.bridge.goto(0.0, 0.0))
        layout.addWidget(centre)

        layout.addStretch(1)

        self.store_button = QPushButton("STORE")
        self.store_button.setCheckable(True)
        self.store_button.setToolTip("Arm, then click a preset slot to save the pose")
        self.store_button.toggled.connect(self._set_store_mode)
        layout.addWidget(self.store_button)

        self.preset_buttons = []
        for index in range(1, 5):
            button = QPushButton(f"P{index}")
            button.setFixedWidth(52)
            button.clicked.connect(lambda _, i=index: self._preset(i))
            layout.addWidget(button)
            self.preset_buttons.append(button)
        return bar

    def _bind_shortcuts(self) -> None:
        step = float(self.bridge.get_parameter("jog_step_deg").value)
        jogs = {
            Qt.Key_Left: (step, 0.0),   # pan is positive to the left
            Qt.Key_Right: (-step, 0.0),
            Qt.Key_Up: (0.0, step),
            Qt.Key_Down: (0.0, -step),
        }
        for key, (dpan, dtilt) in jogs.items():
            QShortcut(QKeySequence(key), self,
                      lambda dp=dpan, dt=dtilt: self._jog(dp, dt))
        QShortcut(QKeySequence("H"), self, self.bridge.home)
        QShortcut(QKeySequence("S"), self, self.store_button.toggle)
        for index in range(1, 5):
            QShortcut(QKeySequence(str(index)), self,
                      lambda i=index: self._preset(i))
        # Fullscreen has to be escapable from the keyboard: with the panels
        # hidden there is no window chrome left to click.
        QShortcut(QKeySequence(Qt.Key_F11), self, self.toggle_fullscreen)
        QShortcut(QKeySequence(Qt.Key_Escape), self, self.leave_fullscreen)

    def toggle_fullscreen(self) -> None:
        if self.isFullScreen():
            self.showMaximized()
        else:
            self.showFullScreen()

    def leave_fullscreen(self) -> None:
        if self.isFullScreen():
            self.showMaximized()

    def present(self) -> None:
        """Show the window at the size this session was configured for."""
        if bool(self.bridge.get_parameter("fullscreen").value):
            self.showFullScreen()
        else:
            self.showMaximized()

    # -- actions -------------------------------------------------------------

    def _jog(self, dpan: float, dtilt: float) -> None:
        self.bridge.goto(self.bridge.pan + dpan, self.bridge.tilt + dtilt)

    def _on_hud_wheel(self, notches: float) -> None:
        step = float(self.bridge.get_parameter("zoom_wheel_step").value)
        self.bridge.set_zoom(self.bridge.zoom + notches * step)

    def _on_hud_click(self, u: float, v: float) -> None:
        frame = self.hud.frame
        aspect = (frame.shape[1] / frame.shape[0]) if frame is not None else 16 / 9
        dpan, dtilt = pixel_to_angles(
            u, v, self.bridge.zoom,
            float(self.bridge.get_parameter("hfov_deg").value),
            aspect,
            float(self.bridge.get_parameter("zoom_max_factor").value),
        )
        self.bridge.goto(self.bridge.pan + dpan, self.bridge.tilt + dtilt)

    def _set_store_mode(self, enabled: bool) -> None:
        self.store_mode = enabled

    def _preset(self, index: int) -> None:
        key = str(index)
        if self.store_mode:
            self.presets[key] = [self.bridge.pan, self.bridge.tilt, self.bridge.zoom]
            self._save_presets()
            self.store_button.setChecked(False)
            self._refresh_preset_labels()
            return
        pose = self.presets.get(key)
        if pose is None:
            return
        self.bridge.goto(pose[0], pose[1], pose[2])

    def _refresh_preset_labels(self) -> None:
        for index, button in enumerate(self.preset_buttons, start=1):
            pose = self.presets.get(str(index))
            button.setToolTip(
                f"pan {pose[0]:+.0f}°  tilt {pose[1]:+.0f}°  zoom {pose[2]*100:.0f}%"
                if pose else "empty - arm STORE and click to save"
            )

    def _load_presets(self) -> dict[str, list[float]]:
        try:
            with open(PRESET_PATH) as handle:
                return json.load(handle)
        except (OSError, ValueError):
            return {}

    def _save_presets(self) -> None:
        try:
            os.makedirs(os.path.dirname(PRESET_PATH), exist_ok=True)
            with open(PRESET_PATH, "w") as handle:
                json.dump(self.presets, handle, indent=2)
        except OSError as exc:
            self.bridge.get_logger().warning(f"could not save presets: {exc}")

    # -- refresh -------------------------------------------------------------

    def _refresh(self) -> None:
        bridge = self.bridge

        if self.video is not None:
            self.hud.set_frame(self.video.latest(), self.video.fps)
            error = getattr(self.video, "error", None)
            if error:
                self.hud.status_text = error

        self.hud.set_state(bridge.pan, bridge.tilt, bridge.zoom)
        self.hud.link = bridge.link_ok()
        self.hud.joy = bridge.joy_ok()
        self.hud.update()

        self.pan_out.set(bridge.pan)
        self.tilt_out.set(bridge.tilt)
        self.zoom_out.set(bridge.zoom * 100.0, "{:.0f}")
        self.rate_out.set(self.video.fps if self.video else 0.0, "{:.0f}")

        self.map.set_state(bridge.pan, bridge.tilt)
        self.joy_monitor.set_joy(bridge.joy_axes, bridge.joy_buttons, bridge.joy_ok())

        if bridge.joy_ok():
            speed = bridge.speed_dps()
            self.joy_status.setText(
                f"live · {speed:.0f}°/s" if speed > 0.5 else "live · holding"
            )
        else:
            self.joy_status.setText("no joystick")

    def closeEvent(self, event) -> None:
        if self.video is not None:
            self.video.close()
        event.accept()


def main(args=None) -> None:
    rclpy.init(args=args)
    bridge = GcsBridge()

    # An explicit executor, rather than rclpy.spin, so the GUI thread has
    # something it can cleanly stop on exit. Destroying a node while a spin is
    # still running aborts the process ("exception not rethrown").
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    # Strip --ros-args and friends so Qt does not choke on them; what is left
    # is still available for Qt's own flags (-style, -platform, ...).
    app = QApplication(remove_ros_args(sys.argv))
    # Ctrl-C in the launch terminal must close the window, not leave Qt's
    # event loop ignoring the signal until launch escalates to SIGTERM.
    # Python only runs the handler between event-loop callbacks, and the
    # 33 ms refresh timer provides those.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    window = GcsWindow(bridge)
    window._refresh_preset_labels()
    window.present()
    code = app.exec_()

    # closeEvent normally releases this, but quitting any other way (a signal,
    # app.quit from elsewhere) would leave the capture thread grabbing frames
    # into interpreter teardown. close() is idempotent.
    if window.video is not None:
        window.video.close()

    executor.shutdown()
    spin.join(timeout=2.0)
    bridge.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
