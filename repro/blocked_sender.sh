#!/bin/bash
# blocked_sender.sh <distro> [deb]
# Wedges a Fast DDS sender for good with Fast DDS alone, in a container of the
# pinned base image: three publishers and one subscriber share a
# shared-memory-only world, and the subscriber dies (SIGKILL) with its
# listeners marked as processing a buffer, which is what a participant killed
# between taking a message and finishing it leaves behind. When a port is
# marked not ok while a send to it is under way (between the send's
# cleanup_output_ports() and the push), the push throws, the sender regenerates
# the port, and the port being a zombie it first runs
# Port::recover_blocked_processing(), whose get_and_remove_blocked_processing()
# calls listener_processing_stop() while it already holds the port's
# empty_cv_mutex: the inner lock times out after 1 s, the slot stays marked,
# and the loop runs again, forever. The sender's thread keeps the participant's
# send lock while it does, so nothing it publishes reaches anyone and a
# restarted subscriber never hears it.
#
# The levers, so that every round takes that path: plant.py writes the
# processing mark into the stopped subscriber's own ports and sets their last
# watchdog check an hour ahead (no watchdog marks them first); after the kill,
# gdb stops the first publisher when it enters
# SharedMemTransport::push_discard() for one of those ports, stops the other
# publishers (the first process to test a dead port for a zombie is the only
# one that can take this path), marks the ports not ok as a watchdog does, and
# lets everything go on. Needs network for apt (gdb) and SYS_PTRACE.
# CASES rounds (default 6), each in a fresh world.
#
# With a .deb, that package is installed first. A round is CUT_OFF when the
# restarted subscriber receives nothing from some live publisher; each round
# also lists the senders that hold a port's mutex with one of their own threads
# (wedge.py). Expected: WEDGED for the stock package and for a package without
# the recover-loop patch (0003), RECOVERED for one with it (read from the
# package changelog). LEVER=flip (the same flip without the processing mark)
# and LEVER=none (a plain kill) are controls: both RECOVER on every package.
#
# Exit 0 when the observed verdict matches EXPECT (default: derived as
# above), 1 when it does not, 2 on a setup error.
# Prints "VERDICT: WEDGED|RECOVERED (<n> of <m> rounds, lever <lever>)".
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
DISTRO=${1:?usage: blocked_sender.sh <distro> [deb]}
DEB=${2:-}
CASES=${CASES:-6}
LEVER=${LEVER:-processing}
DISTROS_ENV=${DISTROS_ENV:-$ROOT/distros.env}
read -r _ PKG DIGEST PIN < <(grep -E "^$DISTRO " "$DISTROS_ENV") || { echo "unknown distro $DISTRO" >&2; exit 2; }
mounts=(-v "$HERE:/repro:ro")
# MIX_DEB: a second package for the uid-10001 participants (repro/install.sh)
[ -n "${MIX_DEB:-}" ] && mounts+=(-v "$(realpath "$MIX_DEB"):/mix/$(basename "$MIX_DEB"):ro")
expect=WEDGED
if [ -n "$DEB" ]; then
  DEB=$(realpath "$DEB"); mounts+=(-v "$DEB:/pkg/$(basename "$DEB"):ro")
  pkg=$(dpkg-deb -f "$DEB" Package)
  dpkg-deb --fsys-tarfile "$DEB" | tar -xO "./usr/share/doc/$pkg/changelog.Debian.gz" 2>/dev/null | zcat \
    | grep -q 'recovering a dead listener' && expect=RECOVERED
fi
[ "$LEVER" = processing ] || expect=RECOVERED
expect=${EXPECT:-$expect}
# SYS_PTRACE: wedge.py reads the senders' mappings, unlinked ports included, through /proc/<pid>/mem.
out=$(docker run --rm --shm-size=2g --cap-add SYS_PTRACE -e DISTRO="$DISTRO" -e PKG="$PKG" -e PIN="${PIN:-}" -e CASES="$CASES" -e LEVER="$LEVER" \
  "${mounts[@]}" "ros:$DISTRO-ros-base@$DIGEST" bash /repro/blocked_inner.sh 2>&1) || { echo "$out"; exit 2; }
echo "$out"
verdict=$(echo "$out" | sed -n 's/^VERDICT: \([A-Z]*\).*/\1/p')
[ "$verdict" = "$expect" ] && { echo "expected $expect: ok"; exit 0; }
echo "expected $expect, got ${verdict:-nothing}"; exit 1
