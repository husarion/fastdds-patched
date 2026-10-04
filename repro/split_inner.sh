#!/bin/bash
# Runs inside ros:<distro>-ros-base (split_world.sh). No set -u: the distro's
# setup script reads unset variables.
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
CC=/opt/ros/$DISTRO/lib/rclcpp_components/component_container
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
# Robot worlds mix users (UGV OS: the driver as root, the airlock half as uid 10001
# with DAC_OVERRIDE): two publishers and one newcomer run as such a user.
AS=(setpriv --reuid=10001 --regid=10001 --clear-groups --inh-caps=+dac_override,+dac_read_search --ambient-caps=+dac_override,+dac_read_search)
newcomer() { timeout -s INT -k 5 30 "$@" ros2 topic list --no-daemon --spin-time 3 2>/dev/null | grep -c '^/w'; }
split=0; broken=0; ran=0
for r in $(seq 1 "$CASES"); do
  ran=$r
  rm -f /dev/shm/fast* /dev/shm/sem.fast* 2>/dev/null
  PIDS=()
  for i in $(seq 1 8); do "$CC" --ros-args -r __node:=cc$i >/dev/null 2>&1 & PIDS+=($!); sleep 0.3; done
  for i in $(seq 1 10); do
    as=(); [ "$i" -ge 9 ] && as=("${AS[@]}")
    "${as[@]}" ros2 topic pub -r 5 "/w$i" std_msgs/msg/String "{data: w$i}" >/dev/null 2>&1 & PIDS+=($!); sleep 0.3
  done
  sleep 6
  python3 /repro/plant.py sem "$DPORT" >/dev/null 2>&1  # dies holding the port's named mutex
  kill -9 "${PIDS[0]}"                                  # the survivors regenerate the port at once
  sleep 27
  s=$(stranded)
  n1=$(newcomer); n2=$(newcomer); u1=$(newcomer "${AS[@]}")
  if [ -n "$s" ]; then split=$((split + 1)); v=SPLIT; else v=whole; fi
  # A newcomer must see every publisher whatever its user: below 10 on both
  # reads, or a uid-10001 newcomer below 10, is a broken world even without a split.
  if [ -z "$s" ] && { [ "$n1" -lt 10 ] && [ "$n2" -lt 10 ] || [ "$u1" -lt 10 ]; }; then broken=$((broken + 1)); v=BROKEN; fi
  echo "round $r: $v stranded=[$s] newcomer sees $n1 and $n2 of 10 publishers, as uid 10001 $u1"
  kill -9 "${PIDS[@]}" 2>/dev/null; wait 2>/dev/null; sleep 1
  # UNTIL_SPLIT=1 (the stock run): one split proves the bug, stop there.
  [ "${UNTIL_SPLIT:-0}" = 1 ] && [ "$split" -gt 0 ] && break
done
if [ "$split" -gt 0 ]; then echo "VERDICT: SPLIT ($split of $ran rounds, $broken more broken)"
elif [ "$broken" -gt 0 ]; then echo "VERDICT: BROKEN ($broken of $ran rounds: newcomers miss publishers without a split)"
else echo "VERDICT: WHOLE (0 of $ran rounds)"; fi
exit 0
