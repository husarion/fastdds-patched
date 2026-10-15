#!/bin/bash
# large_message.sh <distro> [deb]
# Cuts large samples 60 bytes short with Fast DDS alone, in a container of the
# pinned base image, under the robot world's shared-memory-only profile
# (robot_shm_only.xml: a byte-identical copy of ros2_airlock's
# testbed/rig/quadlets/fastdds_shm_only.xml, sha256 b90b78e74075ddf2…, the
# UGV OS robot profile; maxMessageSize 524288). RTPSMessageGroup appends the
# 60-byte statistics submessage only to RTPS messages shorter than 65511 bytes,
# and SharedMemTransport::send() strips it when the message's byte at size - 60
# is 0x80 (the submessage id), whatever the message. Fast DDS 2.14.6 and 2.14.7
# (Jazzy) therefore cut the last 60 bytes off any message of 65511 bytes or more
# whose payload byte there is 0x80. 2.6.12 (Humble) always appends the
# submessage and 3.6.2 (Lyrical) strips it by buffer identity: neither cuts.
#
# Eight cases, each a fresh rclpy publisher and raw subscriber of
# sensor_msgs/Image (20 samples; the byte at size - 60 is data[N - 60]):
#   fill_70000_{reliable,besteffort}  70 000 B of 0x80, one unfragmented DATA each
#   fill_1MiB_{reliable,besteffort}   1 MiB of 0x80, every DATA_FRAG message exposed
#   byte_n60_{reliable,besteffort}    70 000 B of 0x00 with 0x80 at data[N - 60]
#   byte_n61_reliable, byte_n59_reliable  the same byte one position off either way
# A case is WHOLE when every sample arrives with its exact serialized length and
# every field and data byte equal to what was sent (best effort: at least 18 of
# 20, all whole), SHORT when a sample arrives cut short, LOST otherwise.
# Verdict TRUNCATED: byte_n60_reliable delivers all 20 samples exactly 60 bytes
# short while both neighbours arrive whole (the cut is the one byte, not the
# size), and no fill case is whole. WHOLE: all eight cases whole.
#
# Expected: TRUNCATED for a Fast DDS version with the bug (a version whose
# patches/<upstream version>/ carries the 0004 fix) without the fix, WHOLE for
# one with the fix (read from the package changelog) and for an unaffected
# version (Humble's and Lyrical's stock packages).
#
# Exit 0 when the observed verdict matches EXPECT (default: derived as above),
# 1 when it does not, 2 on a setup error.
# Prints "VERDICT: WHOLE|TRUNCATED|MIXED (<reason>)".
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
DISTRO=${1:?usage: large_message.sh <distro> [deb]}
DEB=${2:-}
read -r _ PKG DIGEST PIN < <(grep -E "^$DISTRO " "$ROOT/distros.env") || { echo "unknown distro $DISTRO" >&2; exit 2; }
mounts=(-v "$HERE:/repro:ro")
fixed=0
if [ -n "$DEB" ]; then
  DEB=$(realpath "$DEB"); mounts+=(-v "$DEB:/pkg/$(basename "$DEB"):ro")
  pkg=$(dpkg-deb -f "$DEB" Package)
  dpkg-deb --fsys-tarfile "$DEB" | tar -xO "./usr/share/doc/$pkg/changelog.Debian.gz" 2>/dev/null | zcat \
    | grep -q 'statistics strip finds the statistics submessage' && fixed=1
fi
out=$(docker run --rm --shm-size=1g -e DISTRO="$DISTRO" -e PKG="$PKG" -e PIN="${PIN:-}" "${mounts[@]}" \
  "ros:$DISTRO-ros-base@$DIGEST" bash /repro/large_inner.sh 2>&1) || { echo "$out"; exit 2; }
echo "$out"
ver=$(echo "$out" | sed -n 's/^fastdds: //p')
[ -n "$ver" ] || { echo "no Fast DDS version reported"; exit 2; }
up=${ver%%-*}
expect=WHOLE
if [ "$fixed" = 0 ] && ls "$ROOT/patches/$up/"0004-* >/dev/null 2>&1; then expect=TRUNCATED; fi
expect=${EXPECT:-$expect}
verdict=$(echo "$out" | sed -n 's/^VERDICT: \([A-Z]*\).*/\1/p')
[ "$verdict" = "$expect" ] && { echo "expected $expect: ok"; exit 0; }
echo "expected $expect, got ${verdict:-nothing}"; exit 1
