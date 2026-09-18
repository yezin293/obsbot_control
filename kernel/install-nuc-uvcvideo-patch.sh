#!/bin/bash
# Add the relative-PTZ-speed fix to the librealsense2-dkms uvcvideo on this
# machine (Ubuntu 20.04, kernel 5.15) and rebuild + reload the module.
#
#   sudo bash kernel/install-nuc-uvcvideo-patch.sh
#
# Why: uvcvideo reports V4L2_CID_PAN_SPEED / TILT_SPEED range as [-1, max], so
# any negative (reverse-direction) speed is clamped to -1. The OBSBOT driver's
# velocity mode needs the real -max..max range. Upstream fix landed in 2026-01;
# no Ubuntu kernel ships it yet.
#
# Rollback: sudo bash kernel/install-nuc-uvcvideo-patch.sh --remove
set -euo pipefail

PKG=librealsense2-dkms
VER=1.3.27
SRC=/usr/src/$PKG-$VER
PATCH_NAME=105-obsbot-relative-ptz-speed-5.15.patch
HERE=$(cd "$(dirname "$0")" && pwd)
KVER=$(uname -r)

[[ $EUID -eq 0 ]] || { echo "run with sudo"; exit 1; }
[[ -d $SRC ]] || { echo "$SRC not found -- this script is for the NUC's RealSense DKMS uvcvideo"; exit 1; }
[[ $KVER == 5.15.* ]] || { echo "kernel $KVER is not 5.15 -- patch was prepared for 5.15"; exit 1; }

if [[ ${1:-} == --remove ]]; then
    sed -i "/$PATCH_NAME/d; /^PATCH_MATCH\[20\]=/d" $SRC/dkms.conf
    rm -f $SRC/patches/$PATCH_NAME
    echo "patch unregistered; rebuilding stock RealSense module"
else
    cp "$HERE/uvcvideo-relative-ptz-speed-5.15.patch" $SRC/patches/$PATCH_NAME
    if ! grep -q "$PATCH_NAME" $SRC/dkms.conf; then
        # Register after the last RealSense 5.15 patch (index 19).
        sed -i "/^PATCH_MATCH\[19\]=\"5.15\"/a PATCH[20]=\"$PATCH_NAME\"\nPATCH_MATCH[20]=\"5.15\"" $SRC/dkms.conf
    fi
    grep -n "$PATCH_NAME\|PATCH_MATCH\[20\]" $SRC/dkms.conf
fi

# Keep a copy of the module currently in use.
CUR=/lib/modules/$KVER/updates/dkms/uvcvideo.ko
[[ -f $CUR ]] && cp -n $CUR /root/uvcvideo.ko.before-obsbot-patch && echo "backup: /root/uvcvideo.ko.before-obsbot-patch"

echo "== dkms rebuild ($KVER)"
dkms remove $PKG/$VER -k $KVER || true
dkms build  $PKG/$VER -k $KVER
dkms install $PKG/$VER -k $KVER --force

echo "== reload uvcvideo (close anything using a camera first)"
if fuser -s /dev/video* 2>/dev/null; then
    echo "!! something still has a camera open:"; fuser -v /dev/video* 2>&1 | sed 's/^/   /'
    echo "   close it, then: sudo modprobe -r uvcvideo && sudo modprobe uvcvideo"
    exit 0
fi
modprobe -r uvcvideo && modprobe uvcvideo
sleep 2
echo "== result"
v4l2-ctl -d /dev/video0 --list-ctrls 2>/dev/null | grep -E "pan_speed|tilt_speed" || echo "(no camera on /dev/video0 right now -- replug and run: v4l2-ctl -d /dev/video0 --list-ctrls | grep speed)"
echo "expected: pan_speed min=-160, tilt_speed min=-120"
