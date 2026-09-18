"""ROS 2 driver node for OBSBOT PTZ cameras.

Control model
-------------
The camera has a real velocity interface (UVC PANTILT_RELATIVE, exposed by
V4L2 as the pan/tilt "speed" controls): write a rate in degrees per second and
the gimbal moves at that rate until it is written 0. So this node is thin -- a
joystick deflection becomes a rate, a released stick becomes 0, and the
camera's own position report is published as the state. No integrator, no
setpoint streaming, no estimate of where the camera might be.

Two hardware facts shape everything else in here:

* Velocity commands are only honoured while the camera is **streaming video**,
  and a command sent while it is dozing off after a stream stops can be
  dropped -- including a stop. So the node keeps the camera awake itself
  (`stream: auto`), steps aside when the GCS wants to capture, and refuses to
  command motion until it has seen frames flowing.
* Very occasionally a stop is not acted on. The node reads the position back
  after every stop and re-issues it if the gimbal is still moving.
* Mixing absolute-position commands with velocity confuses the firmware: after
  a velocity move, an absolute command can send one axis to 0 instead of the
  value asked for. So the node never sends absolute positions. `goto` and
  `home` are closed loops on the measured position, driven with velocity.

Topics
------
  cmd_ptz   (geometry_msgs/Twist)   normalised rate command, each field -1..1
                angular.z  pan   (+ = left / counter-clockwise, REP-103)
                angular.y  tilt  (+ = up)
                linear.x   zoom  (+ = in)
  goto_ptz  (geometry_msgs/Vector3) absolute pose: x = pan rad, y = tilt rad,
                z = zoom 0..1. NaN on a field leaves that axis untouched.
  ptz_state (sensor_msgs/JointState) measured pan/tilt in rad, zoom as 0..1

Services
--------
  home           (std_srvs/Trigger)  return to (0, 0) and zoom out
  release_stream (std_srvs/Trigger)  stop the keepalive stream so another
                                     process (the GCS) can capture
"""

from __future__ import annotations

import errno
import math
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .v4l2_ptz import (
    CID_ZOOM_ABSOLUTE,
    PtzDevice,
    PtzError,
    StreamKeepalive,
    find_obsbot,
    rad2deg,
    deg2rad,
)


def _clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


def _slew(current: float, target: float, max_delta: float) -> float:
    """Move `current` toward `target` by at most `max_delta` (0 = jump)."""
    if max_delta <= 0.0:
        return target
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target


# goto: proportional gain (deg/s per degree of error) and the floor rate that
# finishes the last degree without hunting. Position reads are 1-degree
# resolution, so an axis is "there" when the integer error is zero.
_GOTO_GAIN = 3.0
_GOTO_MIN_DPS = 4.0
_GOTO_TIMEOUT_S = 8.0
# After a stop command the gimbal decelerates for ~100 ms; only motion seen
# after this settling window counts as an ignored stop.
_STOP_SETTLE_S = 0.3
_STOP_RETRIES = 3
# Re-send a non-zero velocity this often even if unchanged, so a single dropped
# transfer cannot leave the camera at the wrong rate for long.
_RESEND_S = 0.25


class ObsbotPtzNode(Node):
    def __init__(self) -> None:
        super().__init__("obsbot_ptz")

        self.declare_parameter("device", "")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("frame_id", "obsbot_camera")

        # Full-stick rates, degrees per second. The gimbal is smooth right
        # down to 1 deg/s, so there is no minimum; the joystick's deadzone and
        # expo curve decide how the low end feels.
        self.declare_parameter("max_pan_rate", 60.0)
        self.declare_parameter("max_tilt_rate", 45.0)
        self.declare_parameter("max_zoom_rate", 0.15)  # zoom fraction per second
        # Rate cap for goto / home / click-to-point moves.
        self.declare_parameter("goto_rate", 50.0)

        # Optional ramp, deg/s^2. 0 passes the stick straight through, which
        # is what a stick should feel like; raise it to soften slams.
        self.declare_parameter("pan_accel", 0.0)
        self.declare_parameter("tilt_accel", 0.0)

        self.declare_parameter("invert_pan", False)
        self.declare_parameter("invert_tilt", False)

        # Soft limits in degrees, enforced against the measured position.
        # NaN means "use the hardware limit".
        self.declare_parameter("pan_min", float("nan"))
        self.declare_parameter("pan_max", float("nan"))
        self.declare_parameter("tilt_min", float("nan"))
        self.declare_parameter("tilt_max", float("nan"))

        # Stop if commands go stale: a dropped publisher must not leave the
        # gimbal running into its end stop.
        self.declare_parameter("cmd_timeout", 0.5)

        # Who keeps the camera streaming (velocity commands need it):
        #   auto   - this node, unless another process already streams; the
        #            GCS asks it to step aside via the release_stream service
        #   always - this node, always (headless robot, nothing else captures)
        #   never  - something else must; the node only checks
        self.declare_parameter("stream", "auto")

        device = self.get_parameter("device").value or find_obsbot()
        self.dev = PtzDevice(device)
        if not self.dev.has_pantilt:
            raise PtzError(f"{device} exposes no pan/tilt controls")
        if not self.dev.has_velocity:
            raise PtzError(
                f"{device}: kernel reports the speed controls as [-1, max], so "
                "reverse-direction moves would crawl at 1 deg/s. Install the "
                "uvcvideo patch under kernel/ (see README) and reload the module."
            )

        self._load_limits()

        # A previous driver may have died mid-move: stop first, ask questions later.
        self._safe(self.dev.stop)

        self.keepalive = StreamKeepalive(device)
        self._ka_frames_seen = 0
        self._release_until = 0.0     # keepalive stays off until this time
        self._stream_checked = 0.0
        self.awake = False            # streaming confirmed -> velocity honoured
        self._quiet_until = 0.0       # waking camera may twitch; do not call it a runaway

        self.cmd = (0.0, 0.0, 0.0)    # latest normalised pan/tilt/zoom rate
        self.rate = [0.0, 0.0]        # deg/s actually applied (after ramp)
        self.pos = self._safe(self.dev.get_pantilt_deg) or (0.0, 0.0)
        self.zoom = self.dev.get_zoom() if self.dev.supports(CID_ZOOM_ABSOLUTE) else 0.0
        self.last_cmd_time = self.get_clock().now()

        self._sent = (0, 0)           # velocity last written, deg/s
        self._sent_at = 0.0
        self._stop_at = 0.0           # when we last commanded 0 from motion
        self._stop_check_at = 0.0     # last runaway check while stopped
        self._stop_check_pos = self.pos
        self._stop_retries = 0
        self._target: tuple[float | None, float | None] | None = None  # goto in progress
        self._target_deadline = 0.0
        self._last_zoom: int | None = None
        self._fault_logged = False

        self.create_subscription(
            Twist, "cmd_ptz", self.on_cmd, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.create_subscription(Vector3, "goto_ptz", self.on_goto, 10)
        self.state_pub = self.create_publisher(JointState, "ptz_state", 10)
        self.create_service(Trigger, "home", self.on_home)
        self.create_service(Trigger, "release_stream", self.on_release_stream)

        control_rate = float(self.get_parameter("control_rate").value)
        self.dt = 1.0 / control_rate
        self.create_timer(self.dt, self.on_control_tick)
        self.create_timer(
            1.0 / float(self.get_parameter("publish_rate").value), self.publish_state
        )

        self.get_logger().info(
            f"OBSBOT PTZ ready on {device} | velocity mode | "
            f"pan [{self.pan_min:.0f}, {self.pan_max:.0f}] deg, "
            f"tilt [{self.tilt_min:.0f}, {self.tilt_max:.0f}] deg | "
            f"control {control_rate:.0f} Hz | stream: {self.get_parameter('stream').value} | "
            f"start pose ({self.pos[0]:.1f}, {self.pos[1]:.1f})"
        )

    # -- setup ---------------------------------------------------------------

    def _load_limits(self) -> None:
        hw_pan = self.dev.pan_limits_deg()
        hw_tilt = self.dev.tilt_limits_deg()

        def pick(param: str, fallback: float, low: float, high: float) -> float:
            value = float(self.get_parameter(param).value)
            if math.isnan(value):
                return fallback
            return _clamp(value, low, high)

        self.pan_min = pick("pan_min", hw_pan[0], *hw_pan)
        self.pan_max = pick("pan_max", hw_pan[1], *hw_pan)
        self.tilt_min = pick("tilt_min", hw_tilt[0], *hw_tilt)
        self.tilt_max = pick("tilt_max", hw_tilt[1], *hw_tilt)

    def _safe(self, fn, *args):
        """Run a device call; log a rejected transfer once instead of dying."""
        try:
            result = fn(*args)
        except PtzError as exc:
            if not self._fault_logged:
                self.get_logger().warning(f"gimbal rejected a command: {exc}")
                self._fault_logged = True
            return None
        self._fault_logged = False
        return result

    # -- streaming -----------------------------------------------------------

    def _manage_stream(self, now: float) -> None:
        """Keep `self.awake` true whenever the camera is streaming, by us or not."""
        if now - self._stream_checked < 0.5:
            return
        self._stream_checked = now
        was_awake = self.awake
        self._update_awake(now)
        if self.awake and not was_awake:
            # On waking the camera settles itself for a moment (it has been
            # seen re-centring); give it a few seconds before judging stops.
            self._quiet_until = now + 3.0

    def _update_awake(self, now: float) -> None:
        mode = str(self.get_parameter("stream").value)

        if self.keepalive.running:
            # Frames must actually flow: the camera takes a second to wake up
            # after streamon, and stops delivering if it is unplugged.
            flowing = self.keepalive.frames > self._ka_frames_seen
            self._ka_frames_seen = self.keepalive.frames
            if mode == "never" or now < self._release_until:
                self.keepalive.stop()
                self.awake = False
                self.get_logger().info("keepalive stream stopped")
            else:
                self.awake = flowing
            return

        try:
            elsewhere = self.dev.stream_owned_elsewhere()
        except OSError as exc:
            self.get_logger().warning(f"stream probe failed: {exc.strerror}")
            elsewhere = False
        if elsewhere:
            self.awake = True
            return

        if mode == "never" or now < self._release_until:
            self.awake = False
            return
        try:
            self.keepalive.start()
            self._ka_frames_seen = 0
            self.awake = False  # until frames arrive
            self.get_logger().info("keepalive stream started (nothing else was capturing)")
        except OSError as exc:
            if exc.errno != errno.EBUSY:
                self.get_logger().warning(f"keepalive stream failed: {exc.strerror}")
            self.awake = exc.errno == errno.EBUSY  # someone beat us to it: fine

    def on_release_stream(self, _request, response):
        """The GCS is about to capture: get out of its way for a while."""
        self._release_until = time.monotonic() + 5.0
        self._stream_checked = 0.0
        if self.keepalive.running:
            self._safe(self.dev.stop)
            self.keepalive.stop()
            self.awake = False
            self.get_logger().info("keepalive stream released to another capture client")
        response.success = True
        response.message = "keepalive stream released for 5 s"
        return response

    # -- callbacks -----------------------------------------------------------

    def on_cmd(self, msg: Twist) -> None:
        self.cmd = (
            _clamp(msg.angular.z, -1.0, 1.0),
            _clamp(msg.angular.y, -1.0, 1.0),
            _clamp(msg.linear.x, -1.0, 1.0),
        )
        self.last_cmd_time = self.get_clock().now()
        # The stick always wins: touching it abandons a goto in progress.
        if self._target is not None and (self.cmd[0] != 0.0 or self.cmd[1] != 0.0):
            self._target = None

    def on_goto(self, msg: Vector3) -> None:
        """Move to an absolute pose; NaN fields are left alone."""
        pan = None if math.isnan(msg.x) else _clamp(rad2deg(msg.x), self.pan_min, self.pan_max)
        tilt = None if math.isnan(msg.y) else _clamp(rad2deg(msg.y), self.tilt_min, self.tilt_max)
        if not math.isnan(msg.z):
            self.zoom = _clamp(msg.z, 0.0, 1.0)
            self.cmd = (self.cmd[0], self.cmd[1], 0.0)
            self._push_zoom()
        if pan is not None or tilt is not None:
            self._goto(pan, tilt)

    def on_home(self, _request, response):
        self.zoom = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self._push_zoom()
        self._goto(_clamp(0.0, self.pan_min, self.pan_max),
                   _clamp(0.0, self.tilt_min, self.tilt_max))
        response.success = True
        response.message = "homing to (0, 0), zoom wide"
        return response

    def _goto(self, pan: float | None, tilt: float | None) -> None:
        """Start a closed-loop move; the control tick drives it with velocity."""
        self.cmd = (0.0, 0.0, self.cmd[2])
        self.rate = [0.0, 0.0]
        self._target = (pan, tilt)
        self._target_deadline = time.monotonic() + _GOTO_TIMEOUT_S

    def _goto_velocity(self, now: float) -> tuple[float, float]:
        """Velocity toward the goto target, (0, 0) once both axes are there."""
        if self._target is None:
            return 0.0, 0.0
        if now > self._target_deadline:
            self.get_logger().warning(
                f"goto {self._target} gave up at ({self.pos[0]:+.0f}, {self.pos[1]:+.0f})"
            )
            self._target = None
            return 0.0, 0.0
        cap = float(self.get_parameter("goto_rate").value)
        out = []
        done = True
        for target, current in zip(self._target, self.pos):
            if target is None:
                out.append(0.0)
                continue
            err = target - current
            if abs(err) < 0.5:
                out.append(0.0)
                continue
            done = False
            rate = max(_GOTO_MIN_DPS, min(cap, _GOTO_GAIN * abs(err)))
            out.append(math.copysign(rate, err))
        if done:
            self._target = None
        return out[0], out[1]

    # -- control loop --------------------------------------------------------

    def on_control_tick(self) -> None:
        now = time.monotonic()
        pos = self._safe(self.dev.get_pantilt_deg)
        if pos is not None:
            self.pos = pos

        self._manage_stream(now)

        stale = (
            self.get_clock().now() - self.last_cmd_time
        ).nanoseconds * 1e-9 > float(self.get_parameter("cmd_timeout").value)
        cmd = (0.0, 0.0, 0.0) if stale else self.cmd

        max_pan = float(self.get_parameter("max_pan_rate").value)
        max_tilt = float(self.get_parameter("max_tilt_rate").value)
        pan_sign = -1.0 if self.get_parameter("invert_pan").value else 1.0
        tilt_sign = -1.0 if self.get_parameter("invert_tilt").value else 1.0

        if self._target is not None:
            pan_dps, tilt_dps = self._goto_velocity(now)
            self.rate = [pan_dps, tilt_dps]
        else:
            want_pan = pan_sign * cmd[0] * max_pan
            want_tilt = tilt_sign * cmd[1] * max_tilt
            self.rate[0] = _slew(self.rate[0], want_pan,
                                 float(self.get_parameter("pan_accel").value) * self.dt)
            self.rate[1] = _slew(self.rate[1], want_tilt,
                                 float(self.get_parameter("tilt_accel").value) * self.dt)
            pan_dps, tilt_dps = self.rate
        # Soft limits: never command further past a limit, always allow back.
        if (pan_dps > 0 and self.pos[0] >= self.pan_max) or (pan_dps < 0 and self.pos[0] <= self.pan_min):
            pan_dps = 0.0
        if (tilt_dps > 0 and self.pos[1] >= self.tilt_max) or (tilt_dps < 0 and self.pos[1] <= self.tilt_min):
            tilt_dps = 0.0
        if not self.awake:
            # Asleep, the camera ignores velocity; dozing, it may drop a stop.
            # Send nothing but zero until frames are flowing again.
            pan_dps = tilt_dps = 0.0

        self._write_velocity(int(round(pan_dps)), int(round(tilt_dps)), now)
        self._verify_stop(now)

        max_zoom = float(self.get_parameter("max_zoom_rate").value)
        self.zoom = _clamp(self.zoom + cmd[2] * max_zoom * self.dt, 0.0, 1.0)
        if self.awake:
            self._push_zoom()

    def _write_velocity(self, pan: int, tilt: int, now: float) -> None:
        """Send a velocity when it changes, or periodically while moving."""
        target = (pan, tilt)
        moving = target != (0, 0)
        if target == self._sent and not (moving and now - self._sent_at > _RESEND_S):
            return
        if self._safe(self.dev.set_velocity, float(pan), float(tilt)) is None:
            return
        if not moving and self._sent != (0, 0):
            self._stop_at = now
            self._stop_retries = 0
        self._sent = target
        self._sent_at = now

    def _verify_stop(self, now: float) -> None:
        """Re-issue a stop the gimbal did not act on.

        After a stop the camera decelerates for about 150 ms (up to 5 deg of
        travel from full rate). Motion seen after that, while we are commanding
        zero, means the stop was dropped -- which happens occasionally around
        stream transitions. Re-send it. Checked over 0.2 s windows against a
        baseline taken once the settling window has passed.
        """
        if self._sent != (0, 0) or now - self._stop_at < _STOP_SETTLE_S or now < self._quiet_until:
            self._stop_check_at, self._stop_check_pos = now, self.pos
            return
        if now - self._stop_check_at < 0.2:
            return
        drift = max(abs(self.pos[0] - self._stop_check_pos[0]),
                    abs(self.pos[1] - self._stop_check_pos[1]))
        self._stop_check_at, self._stop_check_pos = now, self.pos
        if drift < 1.0 or self._stop_retries >= _STOP_RETRIES:
            return
        self._stop_retries += 1
        self.get_logger().warning(
            f"gimbal still moving after stop (pose {self.pos[0]:+.0f}, {self.pos[1]:+.0f}); "
            f"re-sending stop ({self._stop_retries}/{_STOP_RETRIES})"
        )
        self._safe(self.dev.stop)

    def _push_zoom(self) -> None:
        """Write zoom only when the register value changes.

        Writing it every tick floods the camera with transfers for a value that
        moves in integer steps; it then lags and overshoots the button.
        """
        if not self.dev.supports(CID_ZOOM_ABSOLUTE):
            return
        meta = self.dev.info(CID_ZOOM_ABSOLUTE)
        step = int(round(meta.minimum + self.zoom * (meta.maximum - meta.minimum)))
        if step != self._last_zoom and self._safe(self.dev.set_zoom, self.zoom) is not None:
            self._last_zoom = step

    # -- state ---------------------------------------------------------------

    def publish_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.name = ["pan", "tilt", "zoom"]
        msg.position = [deg2rad(self.pos[0]), deg2rad(self.pos[1]), self.zoom]
        msg.velocity = [
            deg2rad(self._sent[0]),
            deg2rad(self._sent[1]),
            self.cmd[2] * float(self.get_parameter("max_zoom_rate").value),
        ]
        self.state_pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self.dev.stop()
        except PtzError:
            pass
        self.keepalive.stop()
        self.dev.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObsbotPtzNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
