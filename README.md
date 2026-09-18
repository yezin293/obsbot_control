# obsbot_ptz — ROS 2 PTZ control for OBSBOT cameras

ROS 2 driver, joystick teleop and a ground control station for OBSBOT
pan/tilt/zoom cameras, driven through plain **V4L2 UVC camera-terminal
controls**. No vendor SDK, no reverse-engineered USB payloads, no `OBSBOT
Center` running in the background.

Verified on an **OBSBOT Tiny 2 Lite** (`3564:fef9`) with a **Logitech Extreme
3D Pro** stick, on ROS 2 Foxy (Ubuntu 20.04) and Humble (22.04).

**Needs one kernel patch** for the reverse pan/tilt directions — see
[The kernel patch](#the-kernel-patch). Without it the driver refuses to start
and tells you why.

Two packages, so a headless robot can run the driver without pulling in Qt:

| Package | What it is |
| --- | --- |
| `obsbot_ptz` | driver + joystick teleop. Dependencies: rclpy only |
| `obsbot_gcs` | operator GUI. Adds PyQt5 / OpenCV |

## Why V4L2 and not a vendor library

Most public OBSBOT projects take one of two paths, and both have a cost:

| Approach | Example projects | Problem |
| --- | --- | --- |
| OSC over UDP | `M4ri002/OBSBOT-Tiny-2-lite-control`, `sionirvine/obsbotcontroller` | Needs the OBSBOT Center desktop app running as a relay |
| Reverse-engineered UVC XU | `cgevans/tiny2`, `taxfromdk/obsbot_tiny_reversing`, `lxman/obsbot-mcp` | Per-model magic byte strings that break on firmware updates |
| Official SDK | <https://www.obsbot.com/sdk> | Request-by-email, closed source, C++ |

This camera turned out not to need any of them. It exposes the **standard UVC
camera terminal**, so the in-kernel `uvcvideo` driver already publishes the
gimbal as ordinary V4L2 controls:

```
Pan,  Absolute   ±468000  step 3600  →  ±130°, 1° per step
Tilt, Absolute   ±324000  step 3600  →   ±90°, 1° per step
Zoom, Absolute      0–100
```

Run `ros2 run obsbot_ptz probe` on any OBSBOT to see whether the same holds
for your model.

## How the camera is driven

The camera has a **real velocity interface**. UVC's `PANTILT_RELATIVE`
control, which `uvcvideo` exposes as `Pan (Speed)` / `Tilt (Speed)`, means
"move at this many degrees per second until told 0". Measured: one unit is one
deg/s, exactly linear from 2 to 80, and perfectly smooth at every rate — a
2 deg/s pan shows 0.1 px of frame-to-frame jitter.

So the driver is thin. A joystick deflection becomes a rate, a released stick
becomes 0, and the position the camera itself reports is published as the
state. There is no integrator, no setpoint streaming, no estimate of where
the camera might be.

Three hardware facts shape the rest of it:

**Velocity commands only work while the camera is streaming video.** Asleep,
the gimbal ignores them; worse, a command that lands while it is dozing off
after a stream stops can be dropped — including a stop. So the driver keeps
the camera awake with a small capture stream of its own whenever nothing else
is capturing, steps aside when the GCS opens the camera, and never commands
motion until it has seen frames flowing. Details under
[Who streams](#who-streams).

**Occasionally a stop is not acted on.** The driver reads the position back
at 50 Hz (a real device query, 0.5 ms) and re-issues the stop if the gimbal
is still moving 0.3 s after being told to halt. It logs when that happens.

**Absolute-position commands must not be mixed with velocity.** After a
velocity move, an absolute command can send one axis to 0 instead of where it
was asked to go — reproducibly, but not by any rule worth trusting. So the
driver never sends absolute positions at all: `goto`, `home` and
click-to-point are closed loops on the measured position, driven with
velocity, and land within the camera's 1-degree position resolution.

Position resolution is 1 degree and readback lags real motion by roughly
100 ms. Neither matters for teleop.

## Build

```bash
cd ~/ptz_cam/obsbot_control
colcon build
source install/setup.bash
```

Works on Foxy and Humble; there is nothing distribution-specific in it.
If the machine also has ROS 1 sourced in `.bashrc`, build and run from a
shell where only ROS 2 is sourced — `colcon` bakes whatever it finds in the
environment into `install/setup.bash`.

## The kernel patch

Stock `uvcvideo` derives the minimum of the speed controls from UVC `GET_MIN`,
which is the *slowest* speed the camera supports (1), and reports the range as
`[-1, 160]`. Every negative — reverse-direction — request is then clamped to
-1: "pan left at 40 deg/s" arrives at the camera as "pan left at 1 deg/s".
The fix (report and clamp to `-max`) was merged upstream in January 2026,
tested on an OBSBOT Tiny 2, but no Ubuntu kernel ships it yet.

The driver checks for it at start (`has_velocity`) and refuses to run on an
unpatched kernel rather than crawl in one direction.

[kernel/](kernel/) holds the patch and an installer for the NUC, whose
`uvcvideo` is already a DKMS module (Intel RealSense ships one). It registers
the patch with that package and rebuilds:

```bash
sudo bash kernel/install-nuc-uvcvideo-patch.sh
sudo modprobe -r uvcvideo && sudo modprobe uvcvideo     # with no camera client running
v4l2-ctl -d /dev/video0 --list-ctrls | grep speed        # expect min=-160 / min=-120
```

`--remove` reverts it. Another machine with a stock `uvcvideo` needs the same
28-line change built as its own DKMS module; the patch applies to any 5.x/6.x
`uvc_ctrl.c` with trivial offsets.

## Run

```bash
# picture only, fullscreen — for a screen an audience looks at
ros2 launch obsbot_gcs view.launch.py

# everything an operator needs: driver + joystick + GCS window
ros2 launch obsbot_gcs gcs.launch.py

# GCS without the joystick stack
ros2 launch obsbot_gcs gcs.launch.py joystick:=false

# headless: joystick teleop only, no GUI
ros2 launch obsbot_ptz teleop.launch.py

# driver only, for a tracker or your own GUI to drive
ros2 launch obsbot_ptz ptz.launch.py

# inspect what your camera exposes
ros2 run obsbot_ptz probe
```

## The GCS

Live view with a PTZ head-up display, plus the instruments an operator needs to
know what the gimbal is doing before touching the stick.

| Element | What it tells you |
| --- | --- |
| HUD ladders | travel left on each axis; the marker turns amber in the last 8% before an end stop |
| `LINK` chip | green when `ptz_state` is fresh — the driver is alive |
| `ARMED` / `SAFE` | whether the deadman is held, i.e. whether the stick can move anything |
| POSITION map | both axes at once, with a fading trail of where the camera has been |
| JOYSTICK monitor | live axes, throttle and button LEDs — a mis-mapped axis is obvious immediately |

| Input | Action |
| --- | --- |
| **click the video** | point the camera there (zoom-corrected) |
| **mouse wheel over the video** | zoom in / out |
| arrow keys | jog by `jog_step_deg` |
| `H` | home |
| `1`–`4` | recall preset |
| `S`, then a slot | store preset (persists to `~/.config/obsbot_gcs/presets.json`) |

Wheel zoom is deliberately a zoom-only command, so scrolling mid-shot does not
jerk a pan in progress to a halt.

### Presentation modes

Nothing is drawn over the video by default — the readouts live in the sidebar,
so the picture itself stays clean enough to put in front of an audience. Four
parameters control the framing, and `view.launch.py` sets all of them:

| Parameter | Effect |
| --- | --- |
| `show_overlay` | telemetry and travel ladders drawn on the picture (default off) |
| `show_ladders` | only the pan/tilt travel bars, with the current angle riding on the marker — on in `view.launch.py`, so a screen with no sidebar still shows how far from centre the camera is and when it nears an end stop |
| `show_panels` | sidebar and button bar (default on) |
| `fullscreen` | open fullscreen rather than maximised |

`view.launch.py` runs the **identical stack** — same driver, same joystick,
same click-to-point, presets, wheel and keys — and only changes those.
Functional parameters still come from the same `gcs.yaml`, so the two launch
files cannot drift apart.

**F11** toggles fullscreen and **Esc** leaves it. With the panels hidden there
is no window chrome to click, so keep those in mind before you go fullscreen on
a stadium display.

The GCS **runs alongside** the joystick rather than replacing it — the stick
keeps flying the camera while the window shows you what it is doing.

### Where the GCS gets video

By default it opens the camera directly (lowest latency, no image traffic on
the DDS bus). Control and capture are independent file descriptors, so this
does not disturb the gimbal — measured 1080p30 with zero dropped frames and
zero control errors while panning.

Running the GCS on a *different machine* from the camera? Set `image_topic` in
[config/gcs.yaml](src/obsbot_gcs/config/gcs.yaml) and it subscribes to a
`sensor_msgs/Image` instead. You will need a camera publisher on the robot
(`teleop.launch.py camera:=true` starts `v4l2_camera` and tells the driver
to rely on it).

If clicks consistently overshoot the target, lower `hfov_deg`.

## Interface

| Name | Type | Direction |
| --- | --- | --- |
| `/obsbot/cmd_ptz` | `geometry_msgs/Twist` | in — normalised rate, each field −1..1 |
| `/obsbot/goto_ptz` | `geometry_msgs/Vector3` | in — absolute pose, `NaN` skips an axis; the stick cancels it |
| `/obsbot/ptz_state` | `sensor_msgs/JointState` | out — **measured** `pan`/`tilt` in rad, commanded rate in `velocity`, `zoom` 0..1 |
| `/obsbot/home` | `std_srvs/Trigger` | service — recentre to (0, 0), zoom wide |
| `/obsbot/release_stream` | `std_srvs/Trigger` | service — drop the keepalive stream so the caller can capture |

`cmd_ptz` field mapping follows REP-103: `angular.z` = pan (+ = left),
`angular.y` = tilt (+ = up), `linear.x` = zoom (+ = in).

```bash
# pan left at half speed; must be published repeatedly or the watchdog stops it
ros2 topic pub -r 20 /obsbot/cmd_ptz geometry_msgs/msg/Twist '{angular: {z: 0.5}}'

# jump to an absolute pose
ros2 topic pub --once /obsbot/goto_ptz geometry_msgs/msg/Vector3 '{x: 0.5, y: -0.2, z: 0.0}'
```

### Command watchdog

`cmd_ptz` must be **streamed, not sent once**. If no command arrives for
`cmd_timeout` (default 0.5 s) the gimbal decelerates and holds. A dropped
teleop node must never leave the camera panning into its end stop.

`joy_to_ptz` handles this for you: it republishes the latest stick state at
50 Hz rather than only on joystick events, and applies its own `joy_timeout`
if the stick itself stops reporting.

## Joystick map (Logitech Extreme 3D Pro)

**Buttons are named by the number printed on the stick, starting at 1.** ROS
indexes from 0; that conversion lives in the code rather than in your head.

| Control | Action |
| --- | --- |
| Stick X / Y | pan / tilt — **always live, no button to hold** |
| Button 1 (trigger) | zoom in |
| Button 2 | home — recentre to (0, 0) |
| Button 3 | zoom out |
| every other button | nothing, deliberately |

There is **no deadman**: move the stick and the camera moves. What keeps a
runaway from happening is the driver's command watchdog, not a held button.
Twist-to-zoom and the throttle speed scale are off for the same reason — on a
flight stick both are easy to apply by accident.

Button 1 is the trigger, so a finger resting there zooms in continuously and
will fight the GCS mouse wheel. Move `zoom_in_button` if that bites.

Zoom spans 1x to 4x, measured: register 25 → 1.75x, 50 → 2.50x, 75 → 3.25x,
100 → 4.03x. Exactly linear, so `max_zoom_rate` in fraction-per-second is also
magnification-per-second times three. Zoom, like pan and tilt, only responds
while the camera is streaming.

### Axis signs

`joy_node` already follows REP-103 — **stick left and stick up report
positive** — which is the same sign convention `cmd_ptz` uses for pan-left and
tilt-up. So the defaults are `invert_pan: false` and `invert_tilt: false`, and
the mapping is a straight pass-through. Flip one only if you prefer the
opposite feel on that axis.

For a different stick, run `ros2 topic echo /obsbot/joy`, read off the axis
numbers (0-based, as printed) and button numbers (add 1), and edit
`src/obsbot_ptz/config/joystick.yaml`. The GCS's joystick panel is the fastest
way to check a mapping: the dot should track your hand, and it lists exactly
the buttons that are bound.

### Who streams

Only one process can stream from a V4L2 device, and velocity commands only
work while one does. The driver's `stream` parameter says who:

| `stream` | behaviour |
| --- | --- |
| `auto` (default) | the driver runs a small 640×360 keepalive capture of its own unless something else already streams. When the GCS opens the camera it calls `release_stream`, the driver drops its capture for 5 s, and the GCS takes over. When the GCS exits, the driver notices within a second and resumes |
| `always` | the driver streams unconditionally — a headless robot where nothing will ever capture |
| `never` | something else must (`teleop.launch.py camera:=true` sets this for the `v4l2_camera` node); the driver only checks, and holds still while nobody streams |

While the camera is not confirmed streaming the driver commands nothing but
zero, so a stream hand-off can never leave it running.

## Tuning

Start in [config/ptz.yaml](src/obsbot_ptz/config/ptz.yaml):

| Parameter | Effect |
| --- | --- |
| `max_pan_rate`, `max_tilt_rate` | deg/s at full stick. Hardware maximum 160 / 120; above ~80 the picture is a blur |
| `goto_rate` | cap for click-to-point, presets and home |
| `pan_accel`, `tilt_accel` | deg/s² ramp; `0.0` (default) passes the stick straight through |
| `pan_min`/`pan_max`, `tilt_min`/`tilt_max` | soft limits in degrees, enforced against the measured position; `.nan` uses the hardware limit |
| `cmd_timeout` | watchdog window |
| `stream` | see above |

Joystick feel lives in [config/joystick.yaml](src/obsbot_ptz/config/joystick.yaml)
— `deadzone` and `expo` (0 = linear, 1 = heavily curved; the gimbal is smooth
down to 1 deg/s, so a higher expo buys real precision near centre).

## Layout

```
kernel/
  uvcvideo-relative-ptz-speed-5.15.patch   the uvcvideo fix
  install-nuc-uvcvideo-patch.sh            registers it with the NUC's DKMS uvcvideo

src/obsbot_ptz/
  obsbot_ptz/v4l2_ptz.py         ctypes V4L2 layer: controls, velocity, keepalive stream
  obsbot_ptz/ptz_node.py         driver: stick → velocity, closed-loop goto, stop verification
  obsbot_ptz/joy_to_ptz_node.py  joystick → cmd_ptz
  obsbot_ptz/probe.py            `ros2 run obsbot_ptz probe`
  config/ptz.yaml, config/joystick.yaml
  launch/teleop.launch.py, launch/ptz.launch.py

src/obsbot_gcs/
  obsbot_gcs/gcs_node.py         Qt window + ROS bridge
  obsbot_gcs/hud.py              video widget, HUD overlay, click-to-angle math
  obsbot_gcs/panels.py           position map, joystick monitor, readouts
  obsbot_gcs/video.py            direct V4L2 capture or Image subscription
  config/gcs.yaml
  launch/gcs.launch.py
```

## Troubleshooting

**`Could not load the Qt platform plugin "xcb"` and the GCS dies with exit
code -6.** The pip build of `opencv-python` ships its own Qt and, on import,
redirects `QT_QPA_PLATFORM_PLUGIN_PATH` into the `cv2` package. Those plugins
are built against a different Qt than the system PyQt5, so the first
`QApplication` aborts. `video.py` clears the override at import time, and
`gcs_node.py` imports `video` *before* PyQt5 so it happens early enough — if
you reorder those imports, this comes back.

**`qt.qpa.plugin: Could not find the Qt platform plugin "wayland"`.** Harmless.
Qt looks for a native Wayland plugin, does not find one, and falls back to xcb
through XWayland — which works. To silence it, `sudo apt install qtwayland5`.

**`VIDEOIO(V4L2): backend is generally available but can't be used to capture
by name`.** Also harmless, and no longer printed: OpenCV's V4L2 backend cannot
open by path, so it probes first. `video.py` passes the numeric index for a
plain `/dev/videoN` and skips the probe.

**Zoom overshoots, then will not come back.** Fixed, but worth knowing what it
was: the driver used to write the zoom control on every control tick, 50 USB
transfers a second for a value that only moves in integer steps. The camera
fell behind, kept zooming in after the button was released, and swallowed the
start of every zoom-out. `_push` now writes zoom only when the register value
changes. If you raise `max_zoom_rate` far above 0.15 you can bring it back.

**Zoom does nothing at all.** Zoom is only applied while the camera is
streaming, the same as pan and tilt. With `stream: auto` the driver's
keepalive covers it; with `stream: never`, start the capture node.

**A button fires twice, or the camera will not move at all.** An old launch is
still running. `ros2 node list` showing two `/obsbot/obsbot_ptz` or
`/obsbot/joy_to_ptz` entries means two nodes are both driving the camera.
`joy_node` in particular sometimes survives a Ctrl-C; `pkill joy_node`.

**`kernel reports the speed controls as [-1, max]`** at start-up. The
uvcvideo patch is not installed or not loaded — see
[The kernel patch](#the-kernel-patch).

**The stick does nothing, no errors.** The camera is not streaming, so it
ignores velocity. With `stream: auto` that means the keepalive could not
start — another process holds the device without using it (`fuser -v
/dev/video*`). With `stream: never`, start the capture node.

**`gimbal still moving after stop; re-sending stop`** in the log. The
camera dropped a stop, usually around a stream hand-off; the driver caught it
and re-sent. Once in a while is expected. Constantly means something else is
writing to the camera — a second driver instance, most likely.

### Verifying motion

`ptz_state` is the camera's own position report, 1-degree resolution, about
100 ms behind the picture. For anything finer, measure the image:
`phaseCorrelate` between consecutive frames gives the pixel shift, whose sign
is the opposite of the camera's motion.

```python
(dx, dy), _ = cv2.phaseCorrelate(prev_gray_f64, cur_gray_f64)
# dx > 0: scene slid right, so the camera panned LEFT
```

## What is past the standard interface

The camera also publishes a vendor extension unit that OBSBOT Center drives:

```
Extension Unit, bUnitID 2
guidExtensionCode {9a1e7291-6843-4683-6d92-39bc7906ee49}
bNumControls 19
```

It is reachable from userspace via `UVCIOC_CTRL_QUERY` (selector 8 returns the
ASCII string `Tiny 2 Lite StreamCamera`), and that is where the AI tracking
modes live. The command encoding is unknown; writing invented bytes to a
vendor unit is how a camera ends up in a state no public tool can clear.
`lxman/obsbot-mcp`, `cgevans/tiny2` and `taxfromdk/obsbot_tiny_reversing`
have done some of that work if you need it. Pan, tilt and zoom do not — the
standard interface covers them completely.

## Notes

- Control and video capture use independent file descriptors, so the gimbal
  stays driveable while the camera streams — and must stream for it to move.
- The Tiny 2 Lite reports pan ±130° but physically travels to about ±148°
  before its own end stop.
- Your user needs access to the camera node (`video` group, or the seat ACL
  that `logind` already grants on a desktop login).
- Motorised tracking, face-follow and the other AI modes live behind the
  vendor XU commands and are **not** reachable this way. If you need them,
  that is where the official SDK or a reverse-engineered XU project comes in.
