#!/bin/bash
# Runs inside ros:<distro>-ros-base (closed_world.sh). No set -u: the distro's
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
port_file() { ls /dev/shm/fast*_port"$((7400 + 250 * ROS_DOMAIN_ID))" 2>/dev/null | grep -v '_el$\|_sl$\|mutex' | head -1; }
pub() { ros2 topic pub -r 2 "/$1" std_msgs/msg/String "{data: $1}" >/dev/null 2>&1 & echo $!; }
sees_b() { timeout -s INT -k 5 40 ros2 topic list --no-daemon --spin-time 3 2>/dev/null | grep -qx /b; }
A=$(pub a); sleep 3; B=$(pub b); sleep 3; C=$(pub c); sleep 5
sees_b || { echo "VERDICT: SETUP (a newcomer does not see /b in the healthy world)"; exit 1; }
echo "healthy world: a newcomer sees /b"
kill -9 "$A"; sleep 8
P=$(pub p); sleep 5
i0=$(stat -c %i "$(port_file)")
kill -9 "$P"; sleep 10
[ "$(stat -c %i "$(port_file)")" != "$i0" ] && echo "P's death: noticed (the port regenerated)" || echo "P's death: NOT noticed"
for i in $(seq 1 10); do pub "x$i" >/dev/null; done
t0=$SECONDS
while [ $((SECONDS - t0)) -lt "$WAIT_S" ]; do
  sleep 30
  if ! sees_b; then
    echo "VERDICT: CLOSED (a newcomer stopped seeing /b $((SECONDS - t0)) s after the load)"; exit 0
  fi
done
echo "VERDICT: OPEN (a newcomer still sees /b after $WAIT_S s of load)"
kill "$B" "$C" 2>/dev/null
exit 0
