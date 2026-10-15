#!/bin/bash
# Runs inside ros:<distro>-ros-base (large_message.sh). No set -u: the distro's
# setup script reads unset variables.
set -o pipefail
export DEBIAN_FRONTEND=noninteractive
. /repro/install.sh || exit 1
source "/opt/ros/$DISTRO/setup.bash"
echo "fastdds: $(dpkg-query -W -f='${Version}' "ros-$DISTRO-$PKG")"
export ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
export FASTRTPS_DEFAULT_PROFILES_FILE=/repro/robot_shm_only.xml FASTDDS_DEFAULT_PROFILES_FILE=/repro/robot_shm_only.xml
# One case: <name> <data bytes> <fill> <override index|-> <reliable|besteffort> <count> <hz>.
# Prints "case <name>: <WHOLE|SHORT|LOST> <the subscriber's line>": WHOLE when every
# sample arrived whole (best effort: at least 90 % of them, all whole), SHORT when
# some sample arrived cut short, LOST otherwise.
run_case() {
  local name=$1 n=$2 fill=$3 ov=$4 rel=$5 count=$6 hz=$7 line v floor
  rm -f /dev/shm/fast* /dev/shm/sem.fast* 2>/dev/null
  timeout -k 5 90 python3 /repro/large.py sub "/large_$name" "$n" "$fill" "$ov" "$rel" "$count" "$hz" 40 > /tmp/sub.out 2>&1 &
  local sub=$!
  timeout -k 5 90 python3 /repro/large.py pub "/large_$name" "$n" "$fill" "$ov" "$rel" "$count" "$hz" > /tmp/pub.out 2>&1 &
  local pub=$!
  wait "$pub"; wait "$sub"
  line=$(grep '^received' /tmp/sub.out)
  [ -n "$line" ] || { echo "case $name: SETUP (no subscriber report: $(tail -3 /tmp/sub.out /tmp/pub.out | tr '\n' ' '))"; return; }
  read -r got whole short < <(echo "$line" | sed -n 's/^received \([0-9]*\) of [0-9]*, whole \([0-9]*\), short \([0-9]*\).*/\1 \2 \3/p')
  floor=$count; [ "$rel" = besteffort ] && floor=$(((count * 9 + 9) / 10))
  if [ "$short" -gt 0 ]; then v=SHORT
  elif [ "$whole" = "$got" ] && [ "$got" -ge "$floor" ]; then v=WHOLE
  else v=LOST; fi
  echo "case $name: $v ($rel, $n B, fill 0x$fill, 0x80 at ${ov}) $line"
}
# The data is the Image's last field and the DATA the message's last submessage
# (after the RTPS header and INFO_TS), so the message's byte at size - 60 is
# data[N - 60].
N=70000
M=1048576
{
  run_case fill_70000_reliable   $N 80 - reliable   20 10
  run_case fill_70000_besteffort $N 80 - besteffort 20 10
  run_case fill_1MiB_reliable    $M 80 - reliable   20 5
  run_case fill_1MiB_besteffort  $M 80 - besteffort 20 5
  run_case byte_n60_reliable     $N 00 $((N - 60)) reliable   20 10
  run_case byte_n60_besteffort   $N 00 $((N - 60)) besteffort 20 10
  run_case byte_n61_reliable     $N 00 $((N - 61)) reliable   20 10
  run_case byte_n59_reliable     $N 00 $((N - 59)) reliable   20 10
} | tee /tmp/cases.out
c() { sed -n "s/^case $1: \([A-Z]*\).*/\1/p" /tmp/cases.out; }
all=$(sed -n 's/^case [^:]*: \([A-Z]*\).*/\1/p' /tmp/cases.out | sort -u | tr '\n' ' ')
if [ "$all" = "WHOLE " ] && [ "$(grep -c '^case' /tmp/cases.out)" = 8 ]; then
  echo "VERDICT: WHOLE (8 of 8 cases whole and byte-exact)"
elif [ "$(c byte_n60_reliable)" = SHORT ] \
     && sed -n 's/^case byte_n60_reliable: .*expected length \([0-9]*\), lengths \[(\([0-9]*\), 20)\]$/\1 \2/p' /tmp/cases.out | { read -r e g; [ "$g" = "$((e - 60))" ]; } \
     && [ "$(c byte_n61_reliable)" = WHOLE ] && [ "$(c byte_n59_reliable)" = WHOLE ] \
     && ! c 'fill_[^:]*' | grep -q WHOLE; then
  echo "VERDICT: TRUNCATED (0x80 at data[N-60] cuts every sample 60 bytes short, its neighbours do not; no 0x80 fill case arrives whole)"
else
  echo "VERDICT: MIXED (the cases above fit neither verdict)"
fi
exit 0
