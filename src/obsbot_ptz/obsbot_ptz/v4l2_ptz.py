"""Low-level V4L2 PTZ access for OBSBOT cameras.

Pure ctypes/ioctl -- no external dependencies, no vendor SDK. The OBSBOT Tiny
series exposes the standard UVC camera-terminal controls, so pan/tilt/zoom are
reachable through plain V4L2:

    CT_PANTILT_ABSOLUTE -> V4L2_CID_PAN_ABSOLUTE  / V4L2_CID_TILT_ABSOLUTE
    CT_PANTILT_RELATIVE -> V4L2_CID_PAN_SPEED     / V4L2_CID_TILT_SPEED
    CT_ZOOM_ABSOLUTE    -> V4L2_CID_ZOOM_ABSOLUTE

Absolute pan/tilt units are arc-seconds (3600 units == 1 degree).
"""

from __future__ import annotations

import ctypes
import errno
import fcntl
import glob
import math
import os
from dataclasses import dataclass

# --- V4L2 control ids -------------------------------------------------------

CID_PAN_ABSOLUTE = 0x009A0908
CID_TILT_ABSOLUTE = 0x009A0909
CID_ZOOM_ABSOLUTE = 0x009A090D
CID_ZOOM_CONTINUOUS = 0x009A090F
CID_PAN_SPEED = 0x009A0920
CID_TILT_SPEED = 0x009A0921
CID_FOCUS_AUTO = 0x009A090C
CID_FOCUS_ABSOLUTE = 0x009A090A

ARCSEC_PER_DEG = 3600.0

# Errors the gimbal raises for a move it cannot honour right now (end stop,
# busy re-homing, value out of range). A streaming control loop should log and
# carry on rather than tear down the device.
_SOFT_ERRNOS = frozenset(
    {errno.EIO, errno.EBUSY, errno.ERANGE, errno.EINVAL, errno.EAGAIN}
)

# --- ioctl plumbing ---------------------------------------------------------


class _QueryCtrl(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("name", ctypes.c_char * 32),
        ("minimum", ctypes.c_int32),
        ("maximum", ctypes.c_int32),
        ("step", ctypes.c_int32),
        ("default_value", ctypes.c_int32),
        ("flags", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 2),
    ]


class _Control(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint32), ("value", ctypes.c_int32)]


class _ExtControl(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("id", ctypes.c_uint32),
        ("size", ctypes.c_uint32),
        ("reserved2", ctypes.c_uint32),
        ("value64", ctypes.c_int64),
    ]


class _ExtControls(ctypes.Structure):
    _fields_ = [
        ("which", ctypes.c_uint32),
        ("count", ctypes.c_uint32),
        ("error_idx", ctypes.c_uint32),
        ("request_fd", ctypes.c_int32),
        ("reserved", ctypes.c_uint32 * 1),
        ("controls", ctypes.POINTER(_ExtControl)),
    ]


def _iowr(nr: int, size: int) -> int:
    return (3 << 30) | (size << 16) | (ord("V") << 8) | nr


VIDIOC_QUERYCTRL = _iowr(36, ctypes.sizeof(_QueryCtrl))
VIDIOC_G_CTRL = _iowr(27, ctypes.sizeof(_Control))
VIDIOC_S_CTRL = _iowr(28, ctypes.sizeof(_Control))
VIDIOC_S_EXT_CTRLS = _iowr(72, ctypes.sizeof(_ExtControls))
V4L2_CTRL_FLAG_NEXT_CTRL = 0x80000000
V4L2_CTRL_WHICH_CUR_VAL = 0


@dataclass(frozen=True)
class CtrlInfo:
    """Queried range of a single V4L2 control."""

    id: int
    name: str
    minimum: int
    maximum: int
    step: int
    default: int


class PtzError(RuntimeError):
    pass


def find_obsbot(vendor_id: str = "3564") -> str:
    """Return the /dev/videoN node of the first OBSBOT capture device.

    Picks the node that actually reports the camera-terminal controls, which is
    the capture node rather than the metadata node that shares the same USB id.
    """
    candidates = []
    for node in sorted(glob.glob("/dev/video*")):
        sysfs = f"/sys/class/video4linux/{os.path.basename(node)}/device/../idVendor"
        try:
            with open(sysfs) as fh:
                if fh.read().strip().lower() != vendor_id.lower():
                    continue
        except OSError:
            continue
        candidates.append(node)

    for node in candidates:
        try:
            with PtzDevice(node) as dev:
                if dev.has_pantilt:
                    return node
        except (OSError, PtzError):
            continue
    raise PtzError(
        f"no OBSBOT camera with pan/tilt controls found (vendor {vendor_id}); "
        f"candidates checked: {candidates or 'none'}"
    )


class PtzDevice:
    """Thin, blocking wrapper around the PTZ controls of one V4L2 device.

    Every accessor is a USB control transfer (roughly 1-5 ms), so call these
    from a dedicated thread rather than from a ROS executor callback that also
    has to stay responsive.
    """

    def __init__(self, device: str = "/dev/video2"):
        self.device = device
        self._fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        self._ctrls: dict[int, CtrlInfo] = {}
        self._scan_controls()

    # -- lifecycle ----------------------------------------------------------

    def close(self) -> None:
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1

    def __enter__(self) -> "PtzDevice":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- control discovery --------------------------------------------------

    def _scan_controls(self) -> None:
        qc = _QueryCtrl()
        qc.id = V4L2_CTRL_FLAG_NEXT_CTRL
        while True:
            try:
                fcntl.ioctl(self._fd, VIDIOC_QUERYCTRL, qc)
            except OSError:
                break
            self._ctrls[qc.id] = CtrlInfo(
                id=qc.id,
                name=qc.name.decode(errors="replace"),
                minimum=qc.minimum,
                maximum=qc.maximum,
                step=qc.step or 1,
                default=qc.default_value,
            )
            qc.id |= V4L2_CTRL_FLAG_NEXT_CTRL

    @property
    def controls(self) -> dict[int, CtrlInfo]:
        return dict(self._ctrls)

    def info(self, cid: int) -> CtrlInfo:
        try:
            return self._ctrls[cid]
        except KeyError:
            raise PtzError(f"{self.device} does not expose control 0x{cid:08x}")

    def supports(self, cid: int) -> bool:
        return cid in self._ctrls

    @property
    def has_pantilt(self) -> bool:
        return self.supports(CID_PAN_ABSOLUTE) and self.supports(CID_TILT_ABSOLUTE)

    @property
    def has_speed(self) -> bool:
        return self.supports(CID_PAN_SPEED) and self.supports(CID_TILT_SPEED)

    # -- raw get/set --------------------------------------------------------

    def get(self, cid: int) -> int:
        ctrl = _Control(id=cid)
        try:
            fcntl.ioctl(self._fd, VIDIOC_G_CTRL, ctrl)
        except OSError as exc:
            raise PtzError(f"get 0x{cid:08x} failed: {exc.strerror}") from exc
        return ctrl.value

    def set(self, cid: int, value: int) -> int:
        """Clamp `value` into the control's range, apply it, return what was sent."""
        meta = self.info(cid)
        value = int(max(meta.minimum, min(meta.maximum, value)))
        ctrl = _Control(id=cid, value=value)
        try:
            fcntl.ioctl(self._fd, VIDIOC_S_CTRL, ctrl)
        except OSError as exc:
            # The gimbal NAKs a move it cannot honour right now (end stop, busy
            # re-homing). That is not fatal for a streaming control loop.
            if exc.errno not in _SOFT_ERRNOS:
                raise
            raise PtzError(f"set 0x{cid:08x}={value} rejected: {exc.strerror}") from exc
        return value

    def set_pantilt_deg(self, pan_deg: float, tilt_deg: float) -> tuple[float, float]:
        """Set pan and tilt in a single ioctl.

        UVC carries pan and tilt in one CT_PANTILT_ABSOLUTE transaction; issuing
        two separate S_CTRL calls makes the driver do a read-modify-write twice
        and lets a diagonal move arrive as two staggered axis moves. One
        S_EXT_CTRLS keeps the axes in lockstep.
        """
        pan_meta = self.info(CID_PAN_ABSOLUTE)
        tilt_meta = self.info(CID_TILT_ABSOLUTE)
        pan_raw = int(max(pan_meta.minimum, min(pan_meta.maximum,
                                                round(pan_deg * ARCSEC_PER_DEG))))
        tilt_raw = int(max(tilt_meta.minimum, min(tilt_meta.maximum,
                                                  round(tilt_deg * ARCSEC_PER_DEG))))

        arr = (_ExtControl * 2)()
        arr[0].id, arr[0].size, arr[0].value64 = CID_PAN_ABSOLUTE, 0, pan_raw
        arr[1].id, arr[1].size, arr[1].value64 = CID_TILT_ABSOLUTE, 0, tilt_raw
        req = _ExtControls(
            which=V4L2_CTRL_WHICH_CUR_VAL, count=2, error_idx=0,
            request_fd=0, controls=arr,
        )
        try:
            fcntl.ioctl(self._fd, VIDIOC_S_EXT_CTRLS, req)
        except OSError as exc:
            if exc.errno not in _SOFT_ERRNOS:
                raise
            raise PtzError(
                f"pan/tilt ({pan_deg:.1f}, {tilt_deg:.1f}) rejected at index "
                f"{req.error_idx}: {exc.strerror}"
            ) from exc
        return pan_raw / ARCSEC_PER_DEG, tilt_raw / ARCSEC_PER_DEG

    def set_move_speed(self, pan: int | None = None, tilt: int | None = None) -> None:
        """Set how fast the gimbal slews toward each absolute setpoint.

        This is not a velocity command -- writing it alone moves nothing. It
        shapes the response to `set_pantilt_deg`: a high value tracks streamed
        setpoints crisply, a low value eases between them and hides the driver's
        one-degree setpoint quantisation at the cost of lag.
        """
        if pan is not None and self.supports(CID_PAN_SPEED):
            self.set(CID_PAN_SPEED, pan)
        if tilt is not None and self.supports(CID_TILT_SPEED):
            self.set(CID_TILT_SPEED, tilt)

    # -- position (absolute, degrees) ---------------------------------------

    def pan_limits_deg(self) -> tuple[float, float]:
        meta = self.info(CID_PAN_ABSOLUTE)
        return meta.minimum / ARCSEC_PER_DEG, meta.maximum / ARCSEC_PER_DEG

    def tilt_limits_deg(self) -> tuple[float, float]:
        meta = self.info(CID_TILT_ABSOLUTE)
        return meta.minimum / ARCSEC_PER_DEG, meta.maximum / ARCSEC_PER_DEG

    def get_pan_deg(self) -> float:
        return self.get(CID_PAN_ABSOLUTE) / ARCSEC_PER_DEG

    def get_tilt_deg(self) -> float:
        return self.get(CID_TILT_ABSOLUTE) / ARCSEC_PER_DEG

    def set_pan_deg(self, deg: float) -> float:
        return self.set(CID_PAN_ABSOLUTE, round(deg * ARCSEC_PER_DEG)) / ARCSEC_PER_DEG

    def set_tilt_deg(self, deg: float) -> float:
        return self.set(CID_TILT_ABSOLUTE, round(deg * ARCSEC_PER_DEG)) / ARCSEC_PER_DEG

    def get_pantilt_deg(self) -> tuple[float, float]:
        return self.get_pan_deg(), self.get_tilt_deg()

    # NOTE: there is deliberately no velocity command here. V4L2_CID_PAN_SPEED
    # and V4L2_CID_TILT_SPEED look like a velocity interface, but on the Tiny 2
    # Lite writing them produces no motion at all -- they only parameterise
    # absolute moves (see `set_move_speed`). Continuous motion is produced by
    # integrating a rate into a target angle and streaming absolute setpoints,
    # which is what `ObsbotPtzNode` does.

    # -- zoom ----------------------------------------------------------------

    def get_zoom(self) -> float:
        """Zoom as 0.0 (wide) .. 1.0 (tele)."""
        meta = self.info(CID_ZOOM_ABSOLUTE)
        span = meta.maximum - meta.minimum or 1
        return (self.get(CID_ZOOM_ABSOLUTE) - meta.minimum) / span

    def set_zoom(self, frac: float) -> float:
        meta = self.info(CID_ZOOM_ABSOLUTE)
        span = meta.maximum - meta.minimum
        value = self.set(CID_ZOOM_ABSOLUTE, round(meta.minimum + frac * span))
        return (value - meta.minimum) / (span or 1)


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi
