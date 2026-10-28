#!/bin/bash
# host_id.sh <distro> [deb]
# Splits a shared-memory-only ROS 2 world by changing the host's IPv4 addresses
# between two process starts, in a container of the pinned base image (with
# NET_ADMIN, to add an address to its eth0): a publisher starts, an address is
# added, and a newcomer is asked whether it sees the publisher's topic. Fast DDS
# hashes the IPv4 addresses into the host id each SHM locator carries, once per
# process, and drops SHM locators of another host id, so the newcomer is alone.
# This runs twice, without FASTDDS_HOST_ID and with it set for every process.
# Without the variable the world splits on every package (the variable is opt
# in, 0005 changes nothing by itself). With it, a package without 0005 still
# splits and one with 0005 (read from the package changelog; only Jazzy's 2.14.7
# carries it) stays whole.
#
# Exit 0 when the observed verdict matches EXPECT (default: derived as above),
# 1 when it does not, 2 on a setup error.
# Prints "VERDICT: WHOLE|SPLIT (<reason>)". PLATFORM (such as linux/arm64) runs
# the base image of that architecture.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
DISTRO=${1:?usage: host_id.sh <distro> [deb]}
DEB=${2:-}
read -r _ PKG DIGEST PIN < <(grep -E "^$DISTRO " "$ROOT/distros.env") || { echo "unknown distro $DISTRO" >&2; exit 2; }
mounts=(-v "$HERE:/repro:ro")
expect=SPLIT
if [ -n "$DEB" ]; then
  DEB=$(realpath "$DEB"); mounts+=(-v "$DEB:/pkg/$(basename "$DEB"):ro")
  pkg=$(dpkg-deb -f "$DEB" Package)
  dpkg-deb --fsys-tarfile "$DEB" | tar -xO "./usr/share/doc/$pkg/changelog.Debian.gz" 2>/dev/null | zcat \
    | grep -q 'FASTDDS_HOST_ID fixes the host id' && expect=WHOLE
fi
expect=${EXPECT:-$expect}
out=$(docker run --rm ${PLATFORM:+--platform "$PLATFORM"} --cap-add NET_ADMIN --shm-size=1g -e DISTRO="$DISTRO" -e PKG="$PKG" -e PIN="${PIN:-}" "${mounts[@]}" \
  "ros:$DISTRO-ros-base@$DIGEST" bash /repro/host_id_inner.sh 2>&1) || { echo "$out"; exit 2; }
echo "$out"
verdict=$(echo "$out" | sed -n 's/^VERDICT: \([A-Z]*\).*/\1/p')
[ "$verdict" = "$expect" ] && { echo "expected $expect: ok"; exit 0; }
echo "expected $expect, got ${verdict:-nothing}"; exit 1
