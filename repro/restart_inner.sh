#!/bin/bash
# Runs inside ros:<distro>-ros-base (restart_world.sh). No set -u: the distro's
# setup script reads unset variables. Inputs: DISTRO, CASES, LEVER (natural |
# processing | forced), CHURN (1: short-lived CLI participants killed with SIGTERM and
# SIGKILL during every round), REPIN_FILE (when set, the outer script re-pins
# the container's cpuset while the round runs; this side only records it).
#
# The sequence of the UGV OS release-candidate split of 2026-10-04: a world of
# root participants (idle containers, three 20 Hz publishers, a subscriber),
# an airlock-like "half" (a C++ participant as uid 10001 with DAC_OVERRIDE)
# and, beside it, a side participant (an rclpy subscriber of the three topics,
# as the half's uid): the half is stopped gracefully (SIGTERM) and 0.4 s later
# the side participant dies by SIGKILL, which is what a container's pid
# namespace teardown does to anything exec'd into it; the half then restarts.
# LEVER=processing first SIGSTOPs the side participant and marks its listener
# slots as processing a buffer (what a participant killed between taking a
# message and finishing it leaves); the rest is left to the world's own timing
# (no gdb, no planted watchdog times). LEVER=forced adds blocked_inner.sh's
# in-send flip (below), so every round takes the path 0003 fixes.
set -o pipefail
export DEBIAN_FRONTEND=noninteractive
if ls /pkg/*.deb >/dev/null 2>&1; then
  dpkg -i /pkg/*.deb >/dev/null || { echo "dpkg -i failed"; exit 1; }
  echo "installed: $(dpkg-query -W -f='${Package} ${Version}' "$(dpkg-deb -f /pkg/*.deb Package)")"
else
  echo "stock: $(dpkg-query -W -f='${Package} ${Version}\n' "ros-$DISTRO-fastrtps" "ros-$DISTRO-fastdds" 2>/dev/null)"
fi
source "/opt/ros/$DISTRO/setup.bash"
export ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/repro/shm_only.xml FASTDDS_DEFAULT_PROFILES_FILE=/repro/shm_only.xml
DPORT=$((7400 + 250 * ROS_DOMAIN_ID))
NS=3
CC=/opt/ros/$DISTRO/lib/rclcpp_components/component_container
AS=(setpriv --reuid=10001 --regid=10001 --clear-groups --inh-caps=+dac_override,+dac_read_search --ambient-caps=+dac_override,+dac_read_search)
own_ports() { ls -l "/proc/$1/fd" 2>/dev/null | sed -n 's/.*fast\(rtps\|dds\)_port\([0-9]*\)_el$/\2/p' | sort -u | tr '\n' ' '; }
counts() { cat "$1" 2>/dev/null || echo "-"; }
# pids that map a segment of the discovery port but not the port file itself
stranded() {
  python3 - "$DPORT" <<'PY'
import os, re, sys
port = sys.argv[1]
pat = re.compile(r"/fast(rtps|dds)_port" + port + r"( \(deleted\))?$")
names = [n for n in os.listdir("/dev/shm") if re.fullmatch(r"fast(rtps|dds)_port" + port, n)]
file_ino = os.stat("/dev/shm/" + names[0]).st_ino if names else None
out = []
for pid in filter(str.isdigit, os.listdir("/proc")):
    inos = set()
    try:
        for line in open(f"/proc/{pid}/maps"):
            f = line.split()
            if len(f) >= 6 and pat.search(line.rstrip()):
                inos.add(int(f[4]))
    except OSError:
        continue
    if inos and file_ino not in inos:
        out.append(pid)
print(" ".join(out))
PY
}
# live processes holding a unicast port's _el lock whose segment is gone: the
# RC's "Failed init_port ... open_and_lock_file failed" signature
orphaned() {
  local pid p out=""
  for pid in /proc/[0-9]*; do pid=${pid#/proc/}
    for p in $(own_ports "$pid"); do
      [ -e "/dev/shm/fastrtps_port$p" ] || [ -e "/dev/shm/fastdds_port$p" ] || out+="$pid:$p "
    done
  done
  echo "$out"
}
# every distinct inode of the discovery port file, with the time it appeared
watch_port() {
  local last="" ino
  while :; do
    ino=$(stat -c %i "/dev/shm/fastrtps_port$DPORT" 2>/dev/null || stat -c %i "/dev/shm/fastdds_port$DPORT" 2>/dev/null)
    [ -n "$ino" ] && [ "$ino" != "$last" ] && { echo "$(date +%s.%N | cut -c1-14) $ino" >> /tmp/regen; last=$ino; }
    sleep 0.1
  done
}
# short-lived CLI participants: SIGTERM by timeout, every third one SIGKILLed mid-life
churn() {
  local n=0 p
  while :; do
    n=$((n + 1))
    timeout -s TERM -k 5 5 ros2 topic echo /b1 std_msgs/msg/String >/dev/null 2>&1 & p=$!
    if [ $((n % 3)) = 0 ]; then (sleep "$((1 + RANDOM % 4))"; pkill -9 -P "$p" 2>/dev/null) & fi
    sleep 2
  done
}
# LEVER=forced: the flip of blocked_inner.sh. gdb stops the first publisher when
# it enters SharedMemTransport::push_discard() for one of the dead side
# participant's ports, stops the other publishers, marks those ports not ok (as
# a watchdog does at any moment) and lets everything go on.
if [ "$LEVER" = forced ] || [ "${GDB:-0}" = 1 ]; then
  { apt-get update -qq && apt-get install -y -qq --no-install-recommends gdb; } >/dev/null 2>&1 || { echo "gdb install failed"; exit 1; }
  case "$(uname -m)" in x86_64) ARG3='$rdx' ;; aarch64) ARG3='$x2' ;; *) echo "unsupported arch"; exit 1 ;; esac
fi
flip_in_send() {
  local pid=$1 others=$2; shift 2
  local cond="" p
  for p in "$@"; do cond+="*(unsigned int*)(${ARG3}+4)==$p || "; done
  cat > /tmp/flip.gdb <<G
set pagination off
set confirm off
break eprosima::fastdds::rtps::SharedMemTransport::push_discard
condition 1 ${cond% || }
commands 1
  silent
  printf "flipped in send to port %u\n", *(unsigned int*)(${ARG3}+4)
  shell kill -STOP $others; python3 /repro/plant.py notok $*
  delete 1
  detach
  quit
end
continue
G
  timeout -k 5 60 gdb -q -batch -p "$pid" -x /tmp/flip.gdb 2>/dev/null | grep '^flipped' || echo "no flip"
  sleep 2
  kill -CONT $others
}
stranded_n=0; cut_n=0; orphan_n=0; ran=0
for r in $(seq 1 "$CASES"); do
  ran=$r
  rm -f /dev/shm/fast* /dev/shm/sem.fast* /tmp/c1 /tmp/c2 /tmp/probe /tmp/regen 2>/dev/null
  PIDS=()
  for i in $(seq 1 6); do "$CC" --ros-args -r __node:=cc$i >/dev/null 2>&1 & PIDS+=($!); sleep 0.3; done
  SEND=()
  for i in $(seq 1 $NS); do
    ros2 topic pub -r 20 "/b$i" std_msgs/msg/String "{data: b$i}" >/dev/null 2>&1 & SEND+=($!); sleep 0.3
  done
  python3 /repro/counter.py /tmp/c1 $NS >/dev/null 2>&1 & V=$!
  "${AS[@]}" "$CC" --ros-args -r __node:=half >/dev/null 2>&1 & HALF=$!
  "${AS[@]}" python3 /repro/counter.py /tmp/probe $NS >/dev/null 2>&1 & PROBE=$!
  CH=""; [ "${CHURN:-1}" = 1 ] && { churn & CH=$!; }
  sleep 12
  before=$(counts /tmp/c1); probe_counts=$(counts /tmp/probe)
  ports=$(own_ports "$PROBE")
  if [ -z "$ports" ] || [ "$probe_counts" = "-" ] || echo "$probe_counts" | grep -qw 0; then
    echo "round $r: SETUP probe ports=[$ports] counts=[$probe_counts]"
    [ -n "$CH" ] && kill "$CH"; kill -9 "${PIDS[@]}" "${SEND[@]}" "$V" "$HALF" "$PROBE" 2>/dev/null; pkill -9 -f "topic echo" 2>/dev/null; wait 2>/dev/null; continue
  fi
  watch_port & W=$!
  if [ "$LEVER" = processing ] || [ "$LEVER" = forced ]; then
    kill -STOP "$PROBE"
    # shellcheck disable=SC2086  # one argument per port
    [ "$LEVER" = forced ] && python3 /repro/plant.py nowatch $ports >/dev/null
    # shellcheck disable=SC2086  # one argument per port
    python3 /repro/plant.py processing $ports >/dev/null
  fi
  [ -n "$REPIN_FILE" ] && date +%s > "$REPIN_FILE"   # tells the outer script to re-pin now
  kill -TERM "$HALF"; sleep 0.4; kill -9 "$PROBE"
  wait "$HALF" 2>/dev/null; wait "$PROBE" 2>/dev/null
  flip=""
  # shellcheck disable=SC2086  # one argument per port
  [ "$LEVER" = forced ] && flip=$(flip_in_send "${SEND[0]}" "${SEND[*]:1}" $ports)
  sleep 1
  "${AS[@]}" "$CC" --ros-args -r __node:=half >/dev/null 2>&1 & HALF=$!
  sleep 25
  s1=$(stranded); o1=$(orphaned)
  python3 /repro/counter.py /tmp/c2 $NS >/dev/null 2>&1 & V2=$!
  sleep 20
  s2=$(stranded); o2=$(orphaned)
  after=$(counts /tmp/c2)
  kill "$W" 2>/dev/null
  regen=$(($(wc -l < /tmp/regen) - 1))
  # shellcheck disable=SC2068  # one pid per argument
  wedge=$(python3 /repro/wedge.py ${PIDS[@]} ${SEND[@]} "$V" "$HALF" 2>/dev/null | grep '^SELF' | sed 's/^SELF //' | tr '\n' ';')
  # a process stranded at both reads, 20 s apart
  persist=""; for p in $s1; do echo " $s2 " | grep -q " $p " && persist+="$p "; done
  n0=$(echo "$after" | tr ' ' '\n' | grep -c '^0$'); [ "$after" = "-" ] && n0=$NS
  v=ok
  [ "$n0" -gt 0 ] && { cut_n=$((cut_n + 1)); v=CUT_OFF; }
  [ -n "$persist" ] && { stranded_n=$((stranded_n + 1)); v=STRANDED; }
  persist_o=""; for p in $o1; do echo " $o2 " | grep -q " $p " && persist_o+="$p "; done
  [ -n "$persist_o" ] && orphan_n=$((orphan_n + 1))
  names=""; for p in $persist; do names+="$(cat "/proc/$p/cmdline" 2>/dev/null | tr '\0' ' ' | cut -c1-60);"; done
  if [ -n "$persist" ]; then
    # shellcheck disable=SC2086  # one pid per argument
    python3 /repro/strand.py "$DPORT" $persist 2>&1 | sed 's/^/  | /'
    if [ "${GDB:-0}" = 1 ]; then
      for p in $persist; do timeout -k 5 60 gdb -q -batch -p "$p" -ex "thread apply all bt 12" 2>/dev/null \
        | grep -E '^Thread|^#' | sed 's/^/  | /'; break; done
    fi
  fi
  [ "$flip" = "no flip" ] && v="$v(no flip caught)"
  echo "round $r: $v ${flip:+($flip) }stranded=[$persist] ($names) orphaned_el=[$persist_o] regenerations=$regen counts before [$before] newcomer [$after] self-held [$wedge]"
  [ -n "$CH" ] && kill "$CH" 2>/dev/null
  kill -9 "${PIDS[@]}" "${SEND[@]}" "$V" "$V2" "$HALF" 2>/dev/null; pkill -9 -f "topic echo" 2>/dev/null; pkill -9 -f ros2cli.daemon 2>/dev/null; wait 2>/dev/null; sleep 1
  [ "${UNTIL_HIT:-0}" = 1 ] && [ "$stranded_n" -gt 0 ] && break
done
if [ "$stranded_n" -gt 0 ] || [ "$cut_n" -gt 0 ]; then
  echo "VERDICT: STRANDED ($stranded_n stranded, $cut_n cut off, $orphan_n with an orphaned port lock, of $ran rounds, lever $LEVER)"
else echo "VERDICT: WHOLE (0 of $ran rounds, $orphan_n with an orphaned port lock, lever $LEVER)"; fi
exit 0
