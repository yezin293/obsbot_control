# obsbot_ptz — ROS 2 PTZ control for OBSBOT cameras

ROS 2 Humble driver, joystick teleop and a ground control station for OBSBOT
pan/tilt/zoom cameras, driven through plain **V4L2 UVC camera-terminal
controls**. No vendor SDK, no reverse-engineered USB payloads, no `OBSBOT
Center` running in the background.

Verified on an **OBSBOT Tiny 2 Lite** (`3564:fef9`) with a **Logitech Extreme
3D Pro** stick.

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

## The one hardware surprise

`Pan, Speed` and `Tilt, Speed` look like a velocity interface. **They are not
— writing them moves nothing.** Measured directly: setting `Pan, Speed = 40`
for a full second produced 0.0° of motion. They only set *how fast the gimbal
slews toward an absolute setpoint*.

So the driver owns the integrator. A joystick rate command is acceleration-
limited, integrated into a target angle at 50 Hz, and streamed to the camera as
absolute setpoints — which is what makes a position-only gimbal feel like a
velocity stick.

Two further consequences worth knowing:

- **Position feedback is open-loop.** `uvcvideo` caches the control value
  (a read takes 0.01 ms — it never reaches the camera), so `ptz_state` reports
  the *commanded* pose, not a measured one. Fine for teleop; do not treat it as
  an encoder.
- **Setpoints quantise to 1°.** During slow pans that can show as stepping.
  Lower `move_speed_pan`/`move_speed_tilt` to let the gimbal ease between
  setpoints and smooth it out, at the cost of lag.

Measured headroom: streaming atomic pan+tilt setpoints ran clean at both 30 Hz
and 100 Hz — 0 errors, 0.4 ms median per write, 14 ms worst case.

## Build

```bash
cd ~/obsbot
colcon build --symlink-install
source install/setup.bash
```

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
so the picture itself stays clean enough to put in front of an audience. Three
parameters control the framing, and `view.launch.py` sets all three:

| Parameter | Effect |
| --- | --- |
| `show_overlay` | telemetry and travel ladders drawn on the picture (default off) |
| `show_panels` | sidebar and button bar (default on) |
| `fullscreen` | open fullscreen rather than maximised |

`view.launch.py` runs the **identical stack** — same driver, same joystick,
same click-to-point, presets, wheel and keys — and only changes those three.
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
(`ros-humble-v4l2-camera`) — it is not installed here.

If clicks consistently overshoot the target, lower `hfov_deg`.

## Interface

| Name | Type | Direction |
| --- | --- | --- |
| `/obsbot/cmd_ptz` | `geometry_msgs/Twist` | in — normalised rate, each field −1..1 |
| `/obsbot/goto_ptz` | `geometry_msgs/Vector3` | in — absolute pose, `NaN` skips an axis |
| `/obsbot/ptz_state` | `sensor_msgs/JointState` | out — `pan`/`tilt` in rad, `zoom` 0..1 |
| `/obsbot/home` | `std_srvs/Trigger` | service — recentre to (0, 0), zoom wide |

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
magnification-per-second times three.

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

### Why the camera used to keep moving after you let go

The driver integrates a rate into a target angle open-loop, and the camera
reports no true position to correct against. So whenever the commanded rate
exceeded what the gimbal could actually follow, the target ran ahead and the
difference became a debt — which the camera paid off *after* the stick had
already stopped.

There is a second, sharper version of the same mistake: `_push` compared the
raw arcsecond target to decide whether to write. Pan quantises to one degree,
so a target creeping by 0.02° a tick produced a different number every time
while meaning the identical command, and the driver streamed ~50 redundant
control transfers a second into a camera that then worked through the backlog
late. It now compares the value the device will actually store.

The structural fix is `tracking_rate` + `lead_limit`. The driver keeps an
estimate of where the camera really is — it chases the target at
`tracking_rate` — and never lets the target get more than `lead_limit` ahead
of it. Overrun is then bounded by `lead_limit / tracking_rate` no matter what
is commanded.

Measured, full deflection held for five seconds:

| `tracking_rate` | distance panned | coast after release |
| --- | --- | --- |
| 25 | 185 px | 1.84 s |
| **18** | **542 px** | **0.17 s** |
| 12 | 461 px | 0.16 s |

Note the first row: overdriving made the camera travel *a third* as far,
because the gimbal spent the pan thrashing between setpoints it could never
reach. Faster commands were literally slower. With 18, releasing the stick
stops the picture in ~0.15 s whether you panned for one second or five, and a
`goto` still runs as one fast 64 deg/s move.

### Why slow pans stutter

Two hardware facts combine badly. Pan/tilt setpoints **quantise to 1 degree**
(V4L2 rounds 0.5° to 1°), and the gimbal always slews at **~52 deg/s**
regardless of what `move_speed` is set to — measured, it makes no difference
at 1 or at 160. So a commanded 5 deg/s is physically executed as "dart one
degree in 19 ms, wait 180 ms". Smooth slow motion is not available on this
camera.

`min_pan_rate` / `min_tilt_rate` are the response: the smallest deflection
past the deadzone already commands ~12 deg/s, instead of pretending the range
below it works. Lower them if you want finer framing and can accept the
stepping.

## Tuning

Start in [config/ptz.yaml](src/obsbot_ptz/config/ptz.yaml):

| Parameter | Effect |
| --- | --- |
| `tracking_rate` | how fast the gimbal can follow a *stream* of setpoints. The single most important number here — see below |
| `lead_limit` | how far the target may run ahead of the camera, in degrees |
| `max_pan_rate`, `max_tilt_rate` | deg/s at full stick. Pointless above `tracking_rate` |
| `pan_accel`, `tilt_accel` | deg/s² ramp; lower is smoother, `0.0` is instant and jerky |
| `move_speed_pan`, `move_speed_tilt` | gimbal chase speed: high = crisp, low = smooth but laggy |
| `pan_min`/`pan_max`, `tilt_min`/`tilt_max` | soft limits in degrees; `.nan` uses the hardware limit |
| `cmd_timeout` | watchdog window |

Joystick feel lives in [config/joystick.yaml](src/obsbot_ptz/config/joystick.yaml)
— `deadzone`, `expo` (0 = linear, 1 = heavily curved), and `throttle_min_scale`.

## Layout

```
src/obsbot_ptz/
  obsbot_ptz/v4l2_ptz.py         ctypes V4L2 layer — no dependencies
  obsbot_ptz/ptz_node.py         driver: rate → integrator → absolute setpoints
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
streaming. With no capture client running, writes to `Zoom, Absolute` are
accepted by the ioctl and silently discarded — the register even reads back 0.
Start the GCS (or any capture) and it works.

**A button fires twice, or the camera will not move at all.** An old launch is
still running. `ros2 node list` showing two `/obsbot/obsbot_ptz` or
`/obsbot/joy_to_ptz` entries means two nodes are both driving the camera; the
leftover one also keeps `/dev/video2` open, so the GCS then reports
"cannot open /dev/video2 for capture".

### Verifying motion — do not trust the position readback

`ptz_state` and the V4L2 position controls report the *commanded* pose, not a
measured one (`uvcvideo` caches it; a read never reaches the camera). A test
that only checks those numbers will happily "pass" against a gimbal that never
moved.

To prove real motion, measure the image. `phaseCorrelate` between consecutive
frames gives the pixel shift, whose sign is the opposite of the camera's
motion:

```python
(dx, dy), _ = cv2.phaseCorrelate(prev_gray_f64, cur_gray_f64)
# dx > 0: scene slid right, so the camera panned LEFT
```

Measured this way, with the stick full left: settled −0.5 px, trigger not held
−0.3 px, trigger held +93.0 px.

## The ceiling of the V4L2 approach, and what is past it

Everything above is as smooth as the standard UVC interface can be made. The
two limits — 1-degree setpoints and a fixed ~52 deg/s slew — are properties of
what `uvcvideo` exposes, not of the camera. There is no module parameter or
quirk that unlocks them; the granularity comes from the device's own
`GET_RES`, and V4L2 rounds to it before the value ever leaves the kernel.

The camera does have a finer interface. It publishes a **vendor extension
unit** that OBSBOT Center drives:

```
Extension Unit, bUnitID 2
guidExtensionCode {9a1e7291-6843-4683-6d92-39bc7906ee49}
bNumControls 19
```

It is reachable from userspace without root via `UVCIOC_CTRL_QUERY`. Probing it
read-only shows 22 selectors, 60 bytes each, all GET|SET — and selector 8
returns the ASCII string `Tiny 2 Lite StreamCamera`, which confirms it is the
real vendor channel rather than padding.

That is where smooth velocity control almost certainly lives, along with the AI
tracking modes. What is missing is the command encoding: which selector and
which bytes mean "pan at this rate". Guessing is not an option — writing
invented bytes to a vendor unit is how a camera ends up in a state no public
tool can clear. Decoding it means either a USB capture of OBSBOT Center driving
the gimbal, or lifting the layout from a project that has already done that
work: `lxman/obsbot-mcp`, `cgevans/tiny2`, `taxfromdk/obsbot_tiny_reversing`.

Until then, `min_pan_rate` is the honest workaround rather than a fix.

## Notes

- Control and video capture use independent file descriptors, so the gimbal
  stays driveable while the camera streams.
- Your user needs access to the camera node (`video` group, or the seat ACL
  that `logind` already grants on a desktop login).
- Motorised tracking, face-follow and the other AI modes live behind the
  vendor XU commands and are **not** reachable this way. If you need them,
  that is where the official SDK or a reverse-engineered XU project comes in.
