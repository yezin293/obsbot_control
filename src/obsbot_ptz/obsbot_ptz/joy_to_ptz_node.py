"""Map a joystick onto OBSBOT PTZ rate commands.

Deliberately minimal: the stick pans and tilts, and three buttons do zoom in,
zoom out and home. Nothing else on the joystick does anything -- an unbound
button is simply ignored.

Buttons are named by the number printed on the stick, which starts at 1. ROS
indexes from 0, so the conversion lives here rather than in your head.

Every Joy message is turned into a command immediately, so a stick movement
reaches the driver without waiting for a timer tick. The same command is also
re-emitted at a fixed rate: `joy_node` only publishes when something changes,
so holding the stick at a constant deflection would otherwise produce no
messages and the driver's command watchdog would stop the gimbal mid-pan. A
separate joy-staleness timeout still zeroes the command if the joystick itself
goes away.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


def _shape(value: float, deadzone: float, expo: float) -> float:
    """Apply a deadzone, rescale the remainder to full range, then curve it.

    The expo curve trades resolution near centre for reach at the edges. The
    gimbal is smooth right down to 1 deg/s, so a steeper curve buys genuinely
    fine framing near centre; a flatter one feels more direct.
    """
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    curved = expo * scaled**3 + (1.0 - expo) * scaled
    return sign * min(curved, 1.0)


class JoyToPtzNode(Node):
    def __init__(self) -> None:
        super().__init__("joy_to_ptz")

        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("joy_timeout", 1.0)

        # Logitech Extreme 3D Pro: 0 = stick X, 1 = stick Y, 2 = twist,
        # 3 = throttle slider, 4/5 = hat. Axes stay 0-based because that is
        # how `ros2 topic echo /joy` prints them and they carry no label on
        # the hardware; buttons do, so those are 1-based below.
        self.declare_parameter("pan_axis", 0)
        self.declare_parameter("tilt_axis", 1)
        self.declare_parameter("invert_pan", False)
        self.declare_parameter("invert_tilt", False)

        # Buttons, numbered as printed on the joystick (1 = trigger).
        # 0 disables a binding.
        self.declare_parameter("zoom_in_button", 1)
        self.declare_parameter("home_button", 2)
        self.declare_parameter("zoom_out_button", 3)

        self.declare_parameter("deadzone", 0.10)
        self.declare_parameter("expo", 0.35)
        self.declare_parameter("scale", 1.0)

        self.joy: Joy | None = None
        self.joy_time = self.get_clock().now()
        self._home_pressed = False
        self._warned_axes = False

        self.cmd_pub = self.create_publisher(
            Twist, "cmd_ptz", QoSPresetProfiles.SENSOR_DATA.value
        )
        self.create_subscription(
            Joy, "joy", self.on_joy, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.home_client = self.create_client(Trigger, "home")

        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate, self.on_tick)
        self.get_logger().info(
            f"joy -> cmd_ptz at {rate:.0f} Hz | "
            f"button {self.get_parameter('zoom_in_button').value} zoom in, "
            f"{self.get_parameter('zoom_out_button').value} zoom out, "
            f"{self.get_parameter('home_button').value} home"
        )

    # -- input ---------------------------------------------------------------

    def on_joy(self, msg: Joy) -> None:
        self.joy = msg
        self.joy_time = self.get_clock().now()

        pressed = self._button(msg, "home_button")
        if pressed and not self._home_pressed:
            self._call_home()
        self._home_pressed = pressed
        self._publish()

    def _button(self, msg: Joy, param: str) -> bool:
        """Read a button by its printed number. 0 or unset means unbound."""
        number = int(self.get_parameter(param).value)
        if number < 1:
            return False
        index = number - 1
        if index >= len(msg.buttons):
            return False
        return bool(msg.buttons[index])

    def _axis(self, msg: Joy, index: int) -> float:
        if index < 0:
            return 0.0  # deliberately disabled, not a misconfiguration
        if index >= len(msg.axes):
            if not self._warned_axes:
                self.get_logger().warning(
                    f"axis {index} missing; joystick reports {len(msg.axes)} axes. "
                    "Check the *_axis parameters against `ros2 topic echo /joy`."
                )
                self._warned_axes = True
            return 0.0
        return float(msg.axes[index])

    def _call_home(self) -> None:
        if not self.home_client.service_is_ready():
            self.get_logger().warning("home service unavailable")
            return
        self.home_client.call_async(Trigger.Request())
        self.get_logger().info("home requested")

    # -- output --------------------------------------------------------------

    def on_tick(self) -> None:
        self._publish()

    def _publish(self) -> None:
        cmd = Twist()
        msg = self.joy
        stale = (
            msg is None
            or (self.get_clock().now() - self.joy_time).nanoseconds * 1e-9
            > float(self.get_parameter("joy_timeout").value)
        )
        if stale:
            self.cmd_pub.publish(cmd)  # zeroed: holds position
            return

        deadzone = float(self.get_parameter("deadzone").value)
        expo = float(self.get_parameter("expo").value)
        scale = float(self.get_parameter("scale").value)

        def axis(name: str, invert: str) -> float:
            value = self._axis(msg, int(self.get_parameter(name).value))
            if self.get_parameter(invert).value:
                value = -value
            return _shape(value, deadzone, expo) * scale

        cmd.angular.z = axis("pan_axis", "invert_pan")
        cmd.angular.y = axis("tilt_axis", "invert_tilt")

        zoom_in = self._button(msg, "zoom_in_button")
        zoom_out = self._button(msg, "zoom_out_button")
        if zoom_in != zoom_out:
            cmd.linear.x = scale if zoom_in else -scale
        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyToPtzNode()
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
