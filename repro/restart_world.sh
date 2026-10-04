#!/bin/bash
# restart_world.sh <distro> [deb]
# The UGV OS release-candidate split of 2026-10-04 as a container model, with
# Fast DDS alone: a shared-memory-only world of root participants, an
# airlock-like half (uid 10001 with DAC_OVERRIDE) restarted gracefully, a side
# participant exec'd beside it that dies by SIGKILL 0.4 s into that restart
# (a container's pid namespace teardown), and short-lived `ros2 topic echo`
# participants started every 2 s, ended by timeout's SIGTERM or, every third
# one, by SIGKILL mid-life. restart_inner.sh says what a round does.
#
# LEVER=natural (default) leaves every timing to the world; LEVER=processing
# marks the side participant's listener slots as processing a buffer before it
# dies (what a participant killed between taking a message and finishing it
# leaves; on lynx-165f that kill split 4 of 11 rounds without 0003).
# CHURN=0 drops the CLI participants. REPIN=1 runs `docker update --cpuset-cpus`
# bursts against the container while each round's restart runs (the rt-affinity
# re-pin a UGV OS robot does on every airlock-robot start). CPUS (default 1.5)
# bounds the container like a busy robot; CASES rounds (default 12).
#
# A round is STRANDED when some live process maps only unlinked segments of the
# discovery port at two reads 20 s apart, CUT_OFF when a fresh subscriber
# hears nothing from a live publisher. Each round also reports the discovery
# port's regenerations after the restart, the live processes holding the lock
# of a unicast port whose segment is gone ("orphaned_el", what makes a later
# participant log "Failed init_port ... open_and_lock_file failed"), and the
# senders holding a port mutex with their own thread (wedge.py, the mark of
# the recover loop that 0003 fixes).
#
# LEVER=forced adds the flip of blocked_sender.sh (gdb stops the first
# publisher inside a send to the dead side participant's port and the port is
# marked not ok there; needs network for apt): the RC topology driven down the
# path 0003 fixes every round.
#
# Prints "VERDICT: STRANDED|WHOLE (...)". With LEVER=forced, exit 0 when the
# verdict matches the package (STRANDED without 0003, WHOLE with it, read from
# its changelog; EXPECT overrides), 1 when not; other levers exit 0 (a rate),
# unless EXPECT is set. 2 on a setup error.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
DISTRO=${1:?usage: restart_world.sh <distro> [deb]}
DEB=${2:-}
CASES=${CASES:-12}
LEVER=${LEVER:-natural}
CHURN=${CHURN:-1}
CPUS=${CPUS:-1.5}
REPIN=${REPIN:-0}
DISTROS_ENV=${DISTROS_ENV:-$ROOT/distros.env}
read -r _ _ DIGEST < <(grep -E "^$DISTRO " "$DISTROS_ENV") || { echo "unknown distro $DISTRO" >&2; exit 2; }
mounts=(-v "$HERE:/repro:ro")
expect=STRANDED
if [ -n "$DEB" ]; then
  DEB=$(realpath "$DEB"); mounts+=(-v "$DEB:/pkg/$(basename "$DEB"):ro")
  pkg=$(dpkg-deb -f "$DEB" Package)
  dpkg-deb --fsys-tarfile "$DEB" | tar -xO "./usr/share/doc/$pkg/changelog.Debian.gz" 2>/dev/null | zcat \
    | grep -q 'recovering a dead listener' && expect=WHOLE
fi
# Only the forced lever has a verdict to expect (STRANDED without 0003, WHOLE
# with it); the natural and processing runs measure a rate.
[ "$LEVER" = forced ] && EXPECT=${EXPECT:-$expect}
name=restart-world-$$
tmp=$(mktemp -d); trap 'docker rm -f "$name" >/dev/null 2>&1; rm -rf "$tmp"' EXIT
envs=(-e DISTRO="$DISTRO" -e CASES="$CASES" -e LEVER="$LEVER" -e CHURN="$CHURN" -e UNTIL_HIT="${UNTIL_HIT:-0}" -e GDB="${GDB:-0}")
if [ "$REPIN" = 1 ]; then mounts+=(-v "$tmp:/signal"); envs+=(-e REPIN_FILE=/signal/repin); fi
# SYS_PTRACE: wedge.py reads the senders' mappings, unlinked ports included, through /proc/<pid>/mem.
net=(--network none); { [ "$LEVER" = forced ] || [ "${GDB:-0}" = 1 ]; } && net=()  # gdb comes from apt
docker run -d --name "$name" "${net[@]}" --cpus "$CPUS" --shm-size=2g --cap-add SYS_PTRACE --cap-add DAC_READ_SEARCH "${envs[@]}" \
  "${mounts[@]}" "ros:$DISTRO-ros-base@$DIGEST" bash /repro/restart_inner.sh >/dev/null
if [ "$REPIN" = 1 ]; then
  ncpu=$(nproc)
  while docker inspect -f '{{.State.Running}}' "$name" 2>/dev/null | grep -q true; do
    if [ -s "$tmp/repin" ]; then
      rm -f "$tmp/repin"
      for _ in 1 2 3; do
        docker update --cpuset-cpus "0-$((ncpu - 1))" "$name" >/dev/null 2>&1 || true; sleep 0.3
        docker update --cpuset-cpus "1,2" "$name" >/dev/null 2>&1 || true; sleep 0.3
      done
    fi
    sleep 0.2
  done
fi
docker wait "$name" >/dev/null
out=$(docker logs "$name" 2>&1)
echo "$out"
verdict=$(echo "$out" | sed -n 's/^VERDICT: \([A-Z]*\).*/\1/p')
[ -n "$verdict" ] || exit 2
if [ -n "${EXPECT:-}" ]; then
  [ "$verdict" = "$EXPECT" ] && { echo "expected $EXPECT: ok"; exit 0; }
  echo "expected $EXPECT, got $verdict"; exit 1
fi
exit 0
