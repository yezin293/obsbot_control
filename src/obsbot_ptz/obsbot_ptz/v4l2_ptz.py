"""Low-level V4L2 PTZ access for OBSBOT cameras.

Pure ctypes/ioctl -- no external dependencies, no vendor SDK. The OBSBOT Tiny
series exposes the standard UVC camera-terminal controls, so pan/tilt/zoom are
reachable through plain V4L2:

    CT_PANTILT_ABSOLUTE -> V4L2_CID_PAN_ABSOLUTE  / V4L2_CID_TILT_ABSOLUTE
    CT_PANTILT_RELATIVE -> V4L2_CID_PAN_SPEED     / V4L2_CID_TILT_SPEED
    CT_ZOOM_ABSOLUTE    -> V4L2_CID_ZOOM_ABSOLUTE

Absolute pan/tilt units are arc-seconds (3600 units == 1 degree). The speed
controls are a true velocity interface (UVC PANTILT_RELATIVE): one unit is
one degree per second, and the gimbal keeps moving until it is written 0.

Two hardware facts every caller has to respect:

* Velocity commands are only honoured **while the camera is streaming video**.
  Asleep, the gimbal ignores them; worse, a command sent while it is dozing
  off after a stream stops can be dropped -- including a stop. `StreamKeepalive`
  keeps the camera awake, and `PtzDevice.stream_owned_elsewhere` tells whether
  another process (the GCS) already does.
* Stock uvcvideo reports the speed range as [-1, max] and clamps every negative
  request to -1, so "pan left fast" becomes "pan left at 1 deg/s". The kernel
  patch under kernel/ fixes that; `PtzDevice.has_velocity` checks for it.
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


class _PixFormat(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("pixelformat", ctypes.c_uint32),
        ("field", ctypes.c_uint32),
        ("bytesperline", ctypes.c_uint32),
        ("sizeimage", ctypes.c_uint32),
        ("colorspace", ctypes.c_uint32),
        ("priv", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("ycbcr_enc", ctypes.c_uint32),
        ("quantization", ctypes.c_uint32),
        ("xfer_func", ctypes.c_uint32),
    ]


class _FormatUnion(ctypes.Union):
    _fields_ = [("pix", _PixFormat), ("raw_data", ctypes.c_uint8 * 200)]


class _Format(ctypes.Structure):
    # The union holds pointer-bearing members in the kernel, so it is 8-aligned.
    _fields_ = [("type", ctypes.c_uint32), ("_pad", ctypes.c_uint32), ("fmt", _FormatUnion)]


class _RequestBuffers(ctypes.Structure):
    _fields_ = [
        ("count", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("memory", ctypes.c_uint32),
        ("capabilities", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 1),
    ]


class _Timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]


class _Timecode(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("frames", ctypes.c_uint8),
        ("seconds", ctypes.c_uint8),
        ("minutes", ctypes.c_uint8),
        ("hours", ctypes.c_uint8),
        ("userbits", ctypes.c_uint8 * 4),
    ]


class _BufferM(ctypes.Union):
    _fields_ = [("offset", ctypes.c_uint32), ("userptr", ctypes.c_ulong)]


class _Buffer(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("bytesused", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("field", ctypes.c_uint32),
        ("timestamp", _Timeval),
        ("timecode", _Timecode),
        ("sequence", ctypes.c_uint32),
        ("memory", ctypes.c_uint32),
        ("m", _BufferM),
        ("length", ctypes.c_uint32),
        ("reserved2", ctypes.c_uint32),
        ("request_fd", ctypes.c_int32),
    ]


def _iow(nr: int, size: int) -> int:
    return (1 << 30) | (size << 16) | (ord("V") << 8) | nr


def _iowr(nr: int, size: int) -> int:
    return (3 << 30) | (size << 16) | (ord("V") << 8) | nr


VIDIOC_QUERYCTRL = _iowr(36, ctypes.sizeof(_QueryCtrl))
VIDIOC_G_CTRL = _iowr(27, ctypes.sizeof(_Control))
VIDIOC_S_CTRL = _iowr(28, ctypes.sizeof(_Control))
VIDIOC_G_EXT_CTRLS = _iowr(71, ctypes.sizeof(_ExtControls))
VIDIOC_S_EXT_CTRLS = _iowr(72, ctypes.sizeof(_ExtControls))
VIDIOC_G_FMT = _iowr(4, ctypes.sizeof(_Format))
VIDIOC_S_FMT = _iowr(5, ctypes.sizeof(_Format))
VIDIOC_REQBUFS = _iowr(8, ctypes.sizeof(_RequestBuffers))
VIDIOC_QUERYBUF = _iowr(9, ctypes.sizeof(_Buffer))
VIDIOC_QBUF = _iowr(15, ctypes.sizeof(_Buffer))
VIDIOC_DQBUF = _iowr(17, ctypes.sizeof(_Buffer))
VIDIOC_STREAMON = _iow(18, ctypes.sizeof(ctypes.c_int))
VIDIOC_STREAMOFF = _iow(19, ctypes.sizeof(ctypes.c_int))
V4L2_CTRL_FLAG_NEXT_CTRL = 0x80000000
V4L2_CTRL_WHICH_CUR_VAL = 0
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1
V4L2_FIELD_ANY = 0
V4L2_PIX_FMT_MJPEG = ord("M") | ord("J") << 8 | ord("P") << 16 | ord("G") << 24

# Sanity: these must match the kernel ABI byte for byte or every ioctl fails.
assert ctypes.sizeof(_Format) == 208 and ctypes.sizeof(_Buffer) == 88


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

    def __init__(self, device: str = "/dev/video0"):
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
    def has_velocity(self) -> bool:
        """True when the kernel exposes the full signed speed range.

        Stock uvcvideo reports minimum -1 for the speed controls, which makes
        every reverse-direction command crawl at 1 deg/s. See kernel/.
        """
        if not (self.supports(CID_PAN_SPEED) and self.supports(CID_TILT_SPEED)):
            return False
        return self.info(CID_PAN_SPEED).minimum < -1 and self.info(CID_TILT_SPEED).minimum < -1

    def stream_owned_elsewhere(self) -> bool:
        """Is another file descriptor currently streaming from this device?

        Opens a scratch descriptor and asks for zero buffers: uvcvideo grants
        that only to the one handle allowed to stream, so EBUSY means someone
        else (the GCS, say) holds it. On success the request itself releases
        the privilege again and the descriptor is closed, so nothing changes.
        """
        fd = os.open(self.device, os.O_RDWR | os.O_NONBLOCK)
        try:
            req = _RequestBuffers(count=0, type=V4L2_BUF_TYPE_VIDEO_CAPTURE, memory=V4L2_MEMORY_MMAP)
            fcntl.ioctl(fd, VIDIOC_REQBUFS, req)
            return False
        except OSError as exc:
            if exc.errno == errno.EBUSY:
                return True
            raise
        finally:
            os.close(fd)

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

    def set_velocity(self, pan_dps: float, tilt_dps: float) -> tuple[int, int]:
        """Start moving at the given rates, in degrees per second, until told 0.

        Positive pan is left (counter-clockwise), positive tilt is up -- the
        same sign convention as the absolute position controls and REP-103.
        UVC's own pan-speed sign is the opposite, so it is flipped here once.

        Both axes go in one S_EXT_CTRLS: they live in the same UVC control, so
        one transfer starts a diagonal cleanly instead of two staggered axes.
        Only honoured while the camera is streaming; see the module docstring.
        """
        pan_meta = self.info(CID_PAN_SPEED)
        tilt_meta = self.info(CID_TILT_SPEED)
        pan_raw = int(max(pan_meta.minimum, min(pan_meta.maximum, round(-pan_dps))))
        tilt_raw = int(max(tilt_meta.minimum, min(tilt_meta.maximum, round(tilt_dps))))

        arr = (_ExtControl * 2)()
        arr[0].id, arr[0].size, arr[0].value64 = CID_PAN_SPEED, 0, pan_raw
        arr[1].id, arr[1].size, arr[1].value64 = CID_TILT_SPEED, 0, tilt_raw
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
                f"velocity ({pan_dps:.0f}, {tilt_dps:.0f}) rejected at index "
                f"{req.error_idx}: {exc.strerror}"
            ) from exc
        return -pan_raw, tilt_raw

    def stop(self) -> None:
        """Zero both velocities."""
        self.set_velocity(0.0, 0.0)

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
        """Read the gimbal's own position report, both axes in one transfer.

        This is a real device query (measured 0.3-0.7 ms), not a cached value:
        it tracks the camera through velocity moves and absolute moves alike,
        which is what makes stop verification and soft limits possible.
        """
        arr = (_ExtControl * 2)()
        arr[0].id, arr[0].size = CID_PAN_ABSOLUTE, 0
        arr[1].id, arr[1].size = CID_TILT_ABSOLUTE, 0
        req = _ExtControls(
            which=V4L2_CTRL_WHICH_CUR_VAL, count=2, error_idx=0,
            request_fd=0, controls=arr,
        )
        try:
            fcntl.ioctl(self._fd, VIDIOC_G_EXT_CTRLS, req)
        except OSError as exc:
            raise PtzError(f"position read failed: {exc.strerror}") from exc
        # 32-bit controls fill only the low half of the 64-bit slot.
        pan_raw = ctypes.c_int32(arr[0].value64 & 0xFFFFFFFF).value
        tilt_raw = ctypes.c_int32(arr[1].value64 & 0xFFFFFFFF).value
        return pan_raw / ARCSEC_PER_DEG, tilt_raw / ARCSEC_PER_DEG

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


class StreamKeepalive:
    """Keep the camera streaming so that velocity commands are honoured.

    Runs a minimal V4L2 mmap capture -- two buffers at the smallest MJPEG
    size the camera will give -- on its own descriptor and throws every frame
    away. It exists purely to keep the gimbal awake when nothing else (the
    GCS) is capturing. Only one process can stream at a time, so stop this
    before another capture client opens the device.
    """

    def __init__(self, device: str, width: int = 640, height: int = 360, buffers: int = 2):
        self.device = device
        self._fd = -1
        self._maps: list = []
        self._thread = None
        self._running = False
        self.frames = 0
        self._width, self._height, self._buffers = width, height, buffers

    @property
    def running(self) -> bool:
        return self._running

    def start(self) -> None:
        """Begin streaming. Raises OSError(EBUSY) if someone else already is."""
        if self._running:
            return
        import mmap
        import threading

        fd = os.open(self.device, os.O_RDWR | os.O_NONBLOCK)
        try:
            fmt = _Format(type=V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fmt.fmt.pix.width = self._width
            fmt.fmt.pix.height = self._height
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG
            fmt.fmt.pix.field = V4L2_FIELD_ANY
            fcntl.ioctl(fd, VIDIOC_S_FMT, fmt)

            req = _RequestBuffers(count=self._buffers, type=V4L2_BUF_TYPE_VIDEO_CAPTURE, memory=V4L2_MEMORY_MMAP)
            fcntl.ioctl(fd, VIDIOC_REQBUFS, req)
            maps = []
            for i in range(req.count):
                buf = _Buffer(index=i, type=V4L2_BUF_TYPE_VIDEO_CAPTURE, memory=V4L2_MEMORY_MMAP)
                fcntl.ioctl(fd, VIDIOC_QUERYBUF, buf)
                maps.append(mmap.mmap(fd, buf.length, mmap.MAP_SHARED, mmap.PROT_READ, offset=buf.m.offset))
                fcntl.ioctl(fd, VIDIOC_QBUF, buf)
            fcntl.ioctl(fd, VIDIOC_STREAMON, ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE))
        except BaseException:
            os.close(fd)
            raise

        self._fd, self._maps, self._running = fd, maps, True
        self._thread = threading.Thread(target=self._pump, name="obsbot-keepalive", daemon=True)
        self._thread.start()

    def _pump(self) -> None:
        import select

        buf = _Buffer(type=V4L2_BUF_TYPE_VIDEO_CAPTURE, memory=V4L2_MEMORY_MMAP)
        while self._running:
            readable, _, _ = select.select([self._fd], [], [], 0.5)
            if not readable:
                continue
            try:
                fcntl.ioctl(self._fd, VIDIOC_DQBUF, buf)
                fcntl.ioctl(self._fd, VIDIOC_QBUF, buf)
                self.frames += 1
            except OSError as exc:
                if exc.errno == errno.EAGAIN:
                    continue
                self._running = False  # device went away; stop() cleans up

    def stop(self) -> None:
        if self._fd < 0:
            return
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        try:
            fcntl.ioctl(self._fd, VIDIOC_STREAMOFF, ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE))
        except OSError:
            pass
        for m in self._maps:
            m.close()
        self._maps = []
        try:
            fcntl.ioctl(self._fd, VIDIOC_REQBUFS, _RequestBuffers(count=0, type=V4L2_BUF_TYPE_VIDEO_CAPTURE, memory=V4L2_MEMORY_MMAP))
        except OSError:
            pass
        os.close(self._fd)
        self._fd = -1


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi
