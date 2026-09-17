"""CLI: dump the V4L2 controls an OBSBOT exposes.

Run this first on any new OBSBOT model. Which controls appear -- and their
ranges -- is what decides whether this driver can talk to it at all:

    ros2 run obsbot_ptz probe
    ros2 run obsbot_ptz probe /dev/video2
"""

from __future__ import annotations

import sys

from .v4l2_ptz import (
    ARCSEC_PER_DEG,
    CID_PAN_ABSOLUTE,
    CID_TILT_ABSOLUTE,
    PtzDevice,
    PtzError,
    find_obsbot,
)

_NAMED = {
    CID_PAN_ABSOLUTE: "deg",
    CID_TILT_ABSOLUTE: "deg",
}


def main() -> int:
    try:
        device = sys.argv[1] if len(sys.argv) > 1 else find_obsbot()
    except PtzError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    try:
        dev = PtzDevice(device)
    except OSError as exc:
        print(f"error: cannot open {device}: {exc}", file=sys.stderr)
        return 1

    print(f"device: {device}")
    for cid, meta in dev.controls.items():
        line = (
            f"  0x{cid:08x}  {meta.name:<32} "
            f"min={meta.minimum:<8} max={meta.maximum:<8} "
            f"step={meta.step:<6} default={meta.default}"
        )
        if cid in _NAMED:
            line += (
                f"   [{meta.minimum / ARCSEC_PER_DEG:+.0f}"
                f" .. {meta.maximum / ARCSEC_PER_DEG:+.0f} deg,"
                f" {meta.step / ARCSEC_PER_DEG:.2f} deg/step]"
            )
        print(line)

    print()
    if dev.has_pantilt:
        pan, tilt = dev.get_pantilt_deg()
        print(f"pan/tilt now: {pan:+.1f}, {tilt:+.1f} deg  -- driveable")
    else:
        print("no pan/tilt controls: this device cannot be driven by obsbot_ptz")
    dev.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
