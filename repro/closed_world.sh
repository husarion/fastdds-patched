#!/bin/bash
# closed_world.sh <distro> [deb]
# Closes a shared-memory-only ROS 2 world with Fast DDS alone, in a container
# of the pinned base image: three publishers, the first SIGKILLed (noticed: the
# survivors regenerate the discovery port and keep their old slot index), a
# fourth joins and is SIGKILLed (not noticed), ten more publishers load the
# discovery ring, and newcomers are asked what they see.
# With a .deb, that package is installed first (the patched build): the world
# must stay open. Without one, the stock package: the world must close.
#
# Exit 0 when the observed verdict matches the expectation (CLOSED for stock,
# OPEN for a .deb), 1 when it does not, 2 on a setup error.
# Prints "VERDICT: OPEN|CLOSED (<reason>)".
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
DISTRO=${1:?usage: closed_world.sh <distro> [deb]}
DEB=${2:-}
WAIT_S=${WAIT_S:-600}
read -r _ _ DIGEST < <(grep -E "^$DISTRO " "$ROOT/distros.env") || { echo "unknown distro $DISTRO" >&2; exit 2; }
mounts=(-v "$HERE:/repro:ro")
if [ -n "$DEB" ]; then
  DEB=$(realpath "$DEB"); mounts+=(-v "$DEB:/pkg/$(basename "$DEB"):ro"); expect=OPEN
else
  expect=CLOSED
fi
# --shm-size: each participant maps its own segments; the default 64 MiB is too small.
out=$(docker run --rm --shm-size=1g -e DISTRO="$DISTRO" -e WAIT_S="$WAIT_S" "${mounts[@]}" \
  "ros:$DISTRO-ros-base@$DIGEST" bash /repro/inner.sh 2>&1) || { echo "$out"; exit 2; }
echo "$out"
verdict=$(echo "$out" | sed -n 's/^VERDICT: \([A-Z]*\).*/\1/p')
[ "$verdict" = "$expect" ] && { echo "expected $expect: ok"; exit 0; }
echo "expected $expect, got ${verdict:-nothing}"; exit 1
