#!/bin/bash
# Runs inside ros:<distro>-ros-base with NET_ADMIN (host_id.sh). No set -u: the
# distro's setup script reads unset variables.
set -o pipefail
export DEBIAN_FRONTEND=noninteractive
. /repro/install.sh || exit 1
source "/opt/ros/$DISTRO/setup.bash"
export ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/repro/shm_only.xml FASTDDS_DEFAULT_PROFILES_FILE=/repro/shm_only.xml
command -v ip >/dev/null || { apt-get update -qq && apt-get install -y -qq --no-install-recommends iproute2; } >/dev/null 2>&1
dev=$(ip -o -4 addr show scope global | awk '{print $2; exit}')
[ -n "$dev" ] || { echo "VERDICT: SETUP (no interface with a global IPv4 address)"; exit 1; }
sees_a() { timeout -s INT -k 5 40 ros2 topic list --no-daemon --spin-time 3 2>/dev/null | grep -qx /a; }
# round <label>: a publisher, a newcomer in the same address set (must see it),
# an address added, a newcomer after the change; prints and returns whole/split
round() {
  local A r
  ros2 topic pub -r 2 /a std_msgs/msg/String "{data: a}" >/dev/null 2>&1 & A=$!
  sleep 4
  sees_a || { echo "$1: SETUP (a newcomer does not see /a before the change)"; kill "$A"; return 2; }
  ip addr add 10.213.0.1/24 dev "$dev"
  if sees_a; then r=whole; else r=split; fi
  ip addr del 10.213.0.1/24 dev "$dev"
  kill "$A"; wait "$A" 2>/dev/null
  echo "$1: $r after adding an address to $dev"
  [ "$r" = whole ]
}
round "without FASTDDS_HOST_ID"; rc=$?
[ "$rc" = 2 ] && { echo "VERDICT: SETUP"; exit 1; }
[ "$rc" = 0 ] && { echo "VERDICT: SETUP (whole without FASTDDS_HOST_ID: the address change did not move the host id)"; exit 1; }
export FASTDDS_HOST_ID=0x5a5a
round "with FASTDDS_HOST_ID=$FASTDDS_HOST_ID"; rc=$?
case $rc in
  0) echo "VERDICT: WHOLE (with FASTDDS_HOST_ID the newcomer sees the publisher across the address change)" ;;
  1) echo "VERDICT: SPLIT (with FASTDDS_HOST_ID the newcomer is still alone after the address change)" ;;
  *) echo "VERDICT: SETUP"; exit 1 ;;
esac
