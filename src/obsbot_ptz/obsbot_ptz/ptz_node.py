"""ROS 2 driver node for OBSBOT PTZ cameras.

Control model
-------------
The camera has no working velocity interface (see `v4l2_ptz`), so this node
owns the integrator: an incoming normalised rate is rate-limited, integrated
into a target angle at a fixed control rate, and streamed to the gimbal as
absolute setpoints. That is what makes a joystick feel like a velocity stick
even though the hardware only accepts positions.

Topics
------
  cmd_ptz   (geometry_msgs/Twist)   normalised rate command, each field -1..1
                angular.z  pan   (+ = left / counter-clockwise, REP-103)
                angular.y  tilt  (+ = up)
                linear.x   zoom  (+ = in)
  goto_ptz  (geometry_msgs/Vector3) absolute pose: x = pan rad, y = tilt rad,
                z = zoom 0..1. NaN on a field leaves that axis untouched.
  ptz_state (sensor_msgs/JointState) pan/tilt in rad, zoom as 0..1

Services
--------
  home (std_srvs/Trigger)  return to (0, 0) and zoom out
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .v4l2_ptz import (
    ARCSEC_PER_DEG,
    CID_PAN_ABSOLUTE,
    CID_TILT_ABSOLUTE,
    CID_ZOOM_ABSOLUTE,
    PtzDevice,
    PtzError,
    find_obsbot,
    rad2deg,
    deg2rad,
)


def _clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


def _map_rate(norm: float, min_rate: float, max_rate: float) -> float:
    """Turn a normalised rate into deg/s, skipping the unusable slow range.

    Zero stays zero -- releasing the stick must still mean stop -- but any
    deflection at all starts at `min_rate`, because below it this gimbal
    stutters rather than pans. See min_pan_rate in ObsbotPtzNode.
    """
    if norm == 0.0:
        return 0.0
    sign = 1.0 if norm > 0.0 else -1.0
    span = max(max_rate - min_rate, 0.0)
    return sign * (min_rate + span * min(abs(norm), 1.0))


def _slew(current: float, target: float, max_delta: float) -> float:
    """Move `current` toward `target` by at most `max_delta`."""
    if max_delta <= 0.0:
        return target
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target


class ObsbotPtzNode(Node):
    def __init__(self) -> None:
        super().__init__("obsbot_ptz")

        self.declare_parameter("device", "")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("frame_id", "obsbot_camera")

        # Full-stick speeds. Tune these first if the camera feels sluggish or twitchy.
        self.declare_parameter("max_pan_rate", 60.0)   # deg/s
        self.declare_parameter("max_tilt_rate", 45.0)  # deg/s
        self.declare_parameter("max_zoom_rate", 0.4)   # zoom fraction per second

        # Speed at the smallest deflection past the joystick's deadzone.
        #
        # This exists because of a hard hardware limit: setpoints quantise to
        # 1 degree and the gimbal always slews at ~52 deg/s, so a commanded
        # 5 deg/s is physically executed as "dart 1 degree in 19 ms, wait
        # 180 ms". Slow motion therefore cannot be smooth on this camera --
        # it stutters. Starting at a rate whose steps run together avoids that
        # range entirely rather than pretending it works.
        self.declare_parameter("min_pan_rate", 22.0)   # deg/s
        self.declare_parameter("min_tilt_rate", 18.0)  # deg/s

        # Acceleration limits smooth out stick slams. 0 disables the limiter.
        self.declare_parameter("pan_accel", 240.0)   # deg/s^2
        self.declare_parameter("tilt_accel", 180.0)  # deg/s^2

        # How fast the gimbal can follow a *stream* of setpoints. A single long
        # move runs at ~52 deg/s, but fed one-degree steps it accelerates and
        # decelerates for every one, so sustained tracking is far slower.
        self.declare_parameter("tracking_rate", 18.0)   # deg/s

        # How far the integrated target may run ahead of where the camera can
        # actually be. Without this the target accumulates a debt whenever the
        # commanded rate exceeds `tracking_rate`, and the camera spends it
        # after the stick is released -- which reads as "it moves when I let
        # go". Capping the lead bounds that overrun to lead / tracking_rate.
        self.declare_parameter("lead_limit", 2.0)       # degrees

        self.declare_parameter("invert_pan", False)
        self.declare_parameter("invert_tilt", False)

        # Gimbal slew rate toward each streamed setpoint: high tracks crisply,
        # low eases between setpoints and hides the 1-degree quantisation.
        self.declare_parameter("move_speed_pan", 160)
        self.declare_parameter("move_speed_tilt", 120)

        # Soft limits, in degrees. NaN means "use the hardware limit".
        self.declare_parameter("pan_min", float("nan"))
        self.declare_parameter("pan_max", float("nan"))
        self.declare_parameter("tilt_min", float("nan"))
        self.declare_parameter("tilt_max", float("nan"))

        # Stop moving if commands go stale. Essential for a joystick: a dropped
        # publisher must not leave the gimbal panning into its end stop.
        self.declare_parameter("cmd_timeout", 0.5)

        device = self.get_parameter("device").value or find_obsbot()
        self.dev = PtzDevice(device)
        if not self.dev.has_pantilt:
            raise PtzError(f"{device} exposes no pan/tilt controls")

        self._load_limits()
        self.dev.set_move_speed(
            pan=int(self.get_parameter("move_speed_pan").value),
            tilt=int(self.get_parameter("move_speed_tilt").value),
        )

        # Seed the integrator from the camera's current pose so the first
        # command nudges it from where it actually is.
        self.pan_deg, self.tilt_deg = self.dev.get_pantilt_deg()
        # Estimated true pose, for the lead limiter. Starts on the target.
        self.pan_est, self.tilt_est = self.pan_deg, self.tilt_deg
        self.zoom = self.dev.get_zoom() if self.dev.supports(CID_ZOOM_ABSOLUTE) else 0.0

        self.cmd = (0.0, 0.0, 0.0)  # latest normalised pan/tilt/zoom rate
        self.rate = [0.0, 0.0, 0.0]  # accel-limited rate actually applied
        self.pan_dps = 0.0           # rate actually integrated, deg/s
        self.tilt_dps = 0.0
        self.last_cmd_time = self.get_clock().now()
        self._last_sent: tuple[int, int] | None = None
        self._last_zoom: int | None = None
        self._fault_logged = False

        self.create_subscription(
            Twist, "cmd_ptz", self.on_cmd, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.create_subscription(Vector3, "goto_ptz", self.on_goto, 10)
        self.state_pub = self.create_publisher(JointState, "ptz_state", 10)
        self.create_service(Trigger, "home", self.on_home)

        control_rate = float(self.get_parameter("control_rate").value)
        self.dt = 1.0 / control_rate
        self.create_timer(self.dt, self.on_control_tick)
        self.create_timer(
            1.0 / float(self.get_parameter("publish_rate").value), self.publish_state
        )

        self.get_logger().info(
            f"OBSBOT PTZ ready on {device} | "
            f"pan [{self.pan_min:.0f}, {self.pan_max:.0f}] deg, "
            f"tilt [{self.tilt_min:.0f}, {self.tilt_max:.0f}] deg | "
            f"control {control_rate:.0f} Hz | "
            f"start pose ({self.pan_deg:.1f}, {self.tilt_deg:.1f})"
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

    # -- callbacks -----------------------------------------------------------

    def on_cmd(self, msg: Twist) -> None:
        self.cmd = (
            _clamp(msg.angular.z, -1.0, 1.0),
            _clamp(msg.angular.y, -1.0, 1.0),
            _clamp(msg.linear.x, -1.0, 1.0),
        )
        self.last_cmd_time = self.get_clock().now()

    def on_goto(self, msg: Vector3) -> None:
        """Jump to an absolute pose; NaN fields are left alone."""
        moved_pantilt = False
        if not math.isnan(msg.x):
            self.pan_deg = _clamp(rad2deg(msg.x), self.pan_min, self.pan_max)
            moved_pantilt = True
        if not math.isnan(msg.y):
            self.tilt_deg = _clamp(rad2deg(msg.y), self.tilt_min, self.tilt_max)
            moved_pantilt = True
        if not math.isnan(msg.z):
            self.zoom = _clamp(msg.z, 0.0, 1.0)
            self.rate[2] = 0.0
            self.cmd = (self.cmd[0], self.cmd[1], 0.0)

        # Only abandon the stick's motion on the axes this actually retargets.
        # A zoom-only goto (the GCS mouse wheel) must not stop a pan in
        # progress, or scrolling mid-shot jerks the camera to a halt.
        if moved_pantilt:
            self.rate[0] = self.rate[1] = 0.0
            self.cmd = (0.0, 0.0, self.cmd[2])
            # A goto is a single long move, which the gimbal runs at its full
            # ~52 deg/s. Carry the estimate with it so the lead limiter, which
            # exists for streamed setpoints, does not turn the jump into a crawl.
            self.pan_est, self.tilt_est = self.pan_deg, self.tilt_deg
        self._push()

    def on_home(self, _request, response):
        self.pan_deg = _clamp(0.0, self.pan_min, self.pan_max)
        self.tilt_deg = _clamp(0.0, self.tilt_min, self.tilt_max)
        self.zoom = 0.0
        self.rate = [0.0, 0.0, 0.0]
        self.cmd = (0.0, 0.0, 0.0)
        self.pan_est, self.tilt_est = self.pan_deg, self.tilt_deg
        self._push()
        response.success = True
        response.message = "homed to (0, 0), zoom wide"
        return response

    # -- control loop --------------------------------------------------------

    def on_control_tick(self) -> None:
        stale = (
            self.get_clock().now() - self.last_cmd_time
        ).nanoseconds * 1e-9 > float(self.get_parameter("cmd_timeout").value)
        target = (0.0, 0.0, 0.0) if stale else self.cmd

        pan_accel = float(self.get_parameter("pan_accel").value)
        tilt_accel = float(self.get_parameter("tilt_accel").value)
        max_pan = float(self.get_parameter("max_pan_rate").value)
        max_tilt = float(self.get_parameter("max_tilt_rate").value)
        max_zoom = float(self.get_parameter("max_zoom_rate").value)

        # Accel limits are expressed in deg/s^2, so convert them into the
        # normalised rate space the command lives in before slewing.
        self.rate[0] = _slew(
            self.rate[0], target[0], pan_accel * self.dt / max(max_pan, 1e-6)
        )
        self.rate[1] = _slew(
            self.rate[1], target[1], tilt_accel * self.dt / max(max_tilt, 1e-6)
        )
        self.rate[2] = target[2]

        pan_sign = -1.0 if self.get_parameter("invert_pan").value else 1.0
        tilt_sign = -1.0 if self.get_parameter("invert_tilt").value else 1.0

        pan_dps = _map_rate(
            self.rate[0], float(self.get_parameter("min_pan_rate").value), max_pan
        )
        tilt_dps = _map_rate(
            self.rate[1], float(self.get_parameter("min_tilt_rate").value), max_tilt
        )
        self.pan_dps, self.tilt_dps = pan_dps, tilt_dps

        self.pan_deg = _clamp(
            self.pan_deg + pan_sign * pan_dps * self.dt,
            self.pan_min,
            self.pan_max,
        )
        self.tilt_deg = _clamp(
            self.tilt_deg + tilt_sign * tilt_dps * self.dt,
            self.tilt_min,
            self.tilt_max,
        )
        self.zoom = _clamp(self.zoom + self.rate[2] * max_zoom * self.dt, 0.0, 1.0)

        self._limit_lead()
        self._push()

    def _limit_lead(self) -> None:
        """Keep the target within reach of where the camera can actually be.

        The camera reports no true position, so track an estimate: it chases
        the target at `tracking_rate`, the fastest it manages on a stream of
        setpoints. Clamping the target to within `lead_limit` of that estimate
        stops the integrator building a debt it would otherwise pay off after
        the operator has already let go.
        """
        rate = float(self.get_parameter("tracking_rate").value)
        lead = float(self.get_parameter("lead_limit").value)
        if rate <= 0.0 or lead <= 0.0:
            return
        reach = rate * self.dt

        self.pan_est += _clamp(self.pan_deg - self.pan_est, -reach, reach)
        self.tilt_est += _clamp(self.tilt_deg - self.tilt_est, -reach, reach)

        self.pan_deg = _clamp(self.pan_deg, self.pan_est - lead, self.pan_est + lead)
        self.tilt_deg = _clamp(self.tilt_deg, self.tilt_est - lead, self.tilt_est + lead)

    def _push(self) -> None:
        """Send the current target to the camera, skipping no-op writes.

        The comparison has to be against the value the DEVICE will store, not
        the raw arcsecond target. Pan/tilt quantise to one degree, so a target
        creeping by 0.02 deg a tick produces a different arcsecond number every
        time while meaning the identical command -- which had this streaming
        ~50 redundant control transfers a second. The camera works through that
        backlog after the stick has already stopped, so the pan appears to
        carry on once you let go.
        """
        raw = (
            self._quantise(CID_PAN_ABSOLUTE, self.pan_deg),
            self._quantise(CID_TILT_ABSOLUTE, self.tilt_deg),
        )
        try:
            if raw != self._last_sent:
                self.dev.set_pantilt_deg(self.pan_deg, self.tilt_deg)
                self._last_sent = raw
            if self.dev.supports(CID_ZOOM_ABSOLUTE):
                # Only when the register value actually changes. Writing zoom
                # every tick floods the camera with 50 control transfers a
                # second for a value that moves in integer steps; it then lags
                # behind, carries on zooming after the button is released, and
                # swallows the first part of a zoom-out.
                step = self._zoom_register(self.zoom)
                if step != self._last_zoom:
                    self.dev.set_zoom(self.zoom)
                    self._last_zoom = step
        except PtzError as exc:
            # Throttled: an end stop reports on every tick until the stick moves.
            if not self._fault_logged:
                self.get_logger().warning(f"gimbal rejected a setpoint: {exc}")
                self._fault_logged = True
            return
        self._fault_logged = False

    def _quantise(self, cid: int, degrees: float) -> int:
        """The arcsecond value the camera will actually store for this angle."""
        meta = self.dev.info(cid)
        step = meta.step or 1
        raw = int(max(meta.minimum, min(meta.maximum, round(degrees * ARCSEC_PER_DEG))))
        return round(raw / step) * step

    def _zoom_register(self, frac: float) -> int:
        """The integer the camera will actually store for this zoom fraction."""
        meta = self.dev.info(CID_ZOOM_ABSOLUTE)
        span = meta.maximum - meta.minimum
        return int(round(meta.minimum + frac * span))

    # -- state ---------------------------------------------------------------

    def publish_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.name = ["pan", "tilt", "zoom"]
        msg.position = [deg2rad(self.pan_deg), deg2rad(self.tilt_deg), self.zoom]
        msg.velocity = [
            deg2rad(self.pan_dps),
            deg2rad(self.tilt_dps),
            self.rate[2] * float(self.get_parameter("max_zoom_rate").value),
        ]
        self.state_pub.publish(msg)

    def destroy_node(self) -> bool:
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
