#!/bin/bash
# split_world.sh <distro> [deb]
# Splits a shared-memory-only ROS 2 world with Fast DDS alone, in a container
# of the pinned base image limited to one CPU: eighteen participants share the
# discovery port; a process dies holding the port's named mutex (what a
# participant SIGKILLed while it opens the port leaves behind), then an
# established participant is SIGKILLed, so every survivor regenerates the port
# at once. Each survivor waits on the dead owner's mutex; without the fix they
# all time out together, each resets the mutex for itself, and several of them
# open, remove and create the port at the same time. A process left listening
# on an unlinked segment of the port no longer hears anyone who joins later.
# Each round runs in a fresh world. A stock round splits about one time in
# three (2026-10-04, 24 rounds per release and architecture: the lowest cells
# 6 of 24, single 8-round runs as low as 0 of 8), so the run that expects SPLIT
# stops at the first split and gives up after CASES rounds (default 48:
# 0.75^48 < 1e-5, and still below 1 % at a per-round rate of 0.10, the 95 %
# lower bound of a 6-of-24 cell). The run that expects WHOLE runs all CASES
# rounds (default 8) and passes only with no split and no broken round.
#
# With a .deb, that package is installed first. A round SPLITS when, 27 s
# after the kill, some process maps a segment of the discovery port but not the
# port file. Two publishers and a third newcomer run as uid 10001 with
# DAC_OVERRIDE, as an airlock half joins a root robot world; a round where
# newcomers miss publishers without a split is BROKEN. Expected: SPLIT for the
# stock package and for a package without the port-mutex patch (0002), WHOLE
# for one with it (read from the package's changelog).
#
# Exit 0 when the observed verdict matches EXPECT (default: derived from the
# package version as above), 1 when it does not, 2 on a setup error.
# Prints "VERDICT: SPLIT|BROKEN|WHOLE (<n> of <m> rounds)".
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
DISTRO=${1:?usage: split_world.sh <distro> [deb]}
DEB=${2:-}
read -r _ _ DIGEST < <(grep -E "^$DISTRO " "$ROOT/distros.env") || { echo "unknown distro $DISTRO" >&2; exit 2; }
mounts=(-v "$HERE:/repro:ro")
expect=SPLIT
if [ -n "$DEB" ]; then
  DEB=$(realpath "$DEB"); mounts+=(-v "$DEB:/pkg/$(basename "$DEB"):ro")
  # WHOLE when the package carries the port-mutex fix: build.sh writes every
  # patch's subject into the package changelog.
  pkg=$(dpkg-deb -f "$DEB" Package)
  dpkg-deb --fsys-tarfile "$DEB" | tar -xO "./usr/share/doc/$pkg/changelog.Debian.gz" 2>/dev/null | zcat \
    | grep -q 'only one process resets a port mutex' && expect=WHOLE
fi
expect=${EXPECT:-$expect}
until_split=0; [ "$expect" = SPLIT ] && until_split=1
if [ "$until_split" = 1 ]; then CASES=${CASES:-48}; else CASES=${CASES:-8}; fi
out=$(docker run --rm --cpus 1 --shm-size=2g --cap-add DAC_READ_SEARCH -e DISTRO="$DISTRO" -e CASES="$CASES" -e UNTIL_SPLIT="$until_split" "${mounts[@]}" \
  "ros:$DISTRO-ros-base@$DIGEST" bash /repro/split_inner.sh 2>&1) || { echo "$out"; exit 2; }
echo "$out"
verdict=$(echo "$out" | sed -n 's/^VERDICT: \([A-Z]*\).*/\1/p')
[ "$verdict" = "$expect" ] && { echo "expected $expect: ok"; exit 0; }
echo "expected $expect, got ${verdict:-nothing}"; exit 1
