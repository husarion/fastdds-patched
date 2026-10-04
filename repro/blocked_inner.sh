#!/bin/bash
# Runs inside ros:<distro>-ros-base (blocked_sender.sh). No set -u: the
# distro's setup script reads unset variables. Inputs: DISTRO, CASES, LEVER:
#   processing  the subscriber dies with its listeners marked processing a
#               buffer; its ports are flipped not ok while the first publisher
#               is inside a send to one of them (the proof)
#   flip        the same flip without the processing mark (control)
#   none        a plain SIGKILL of the subscriber (control)
set -o pipefail
export DEBIAN_FRONTEND=noninteractive
if ls /pkg/*.deb >/dev/null 2>&1; then
  dpkg -i /pkg/*.deb >/dev/null || { echo "dpkg -i failed"; exit 1; }
  echo "installed: $(dpkg-query -W -f='${Package} ${Version}' "$(dpkg-deb -f /pkg/*.deb Package)")"
else
  echo "stock: $(dpkg-query -W -f='${Package} ${Version}\n' "ros-$DISTRO-fastrtps" "ros-$DISTRO-fastdds" 2>/dev/null)"
fi
if [ "$LEVER" != none ]; then
  { apt-get update -qq && apt-get install -y -qq --no-install-recommends gdb; } >/dev/null 2>&1 || { echo "gdb install failed"; exit 1; }
fi
source "/opt/ros/$DISTRO/setup.bash"
export ROS_DOMAIN_ID=78 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/repro/shm_only.xml FASTDDS_DEFAULT_PROFILES_FILE=/repro/shm_only.xml
NS=3            # publishers
case "$(uname -m)" in x86_64) ARG3='$rdx' ;; aarch64) ARG3='$x2' ;; *) echo "unsupported arch"; exit 1 ;; esac
CC=/opt/ros/$DISTRO/lib/rclcpp_components/component_container
# the ports a process listens on exclusively (its unicast ports): it holds their _el lock files open
own_ports() { ls -l "/proc/$1/fd" 2>/dev/null | sed -n 's/.*fast\(rtps\|dds\)_port\([0-9]*\)_el$/\2/p' | sort -u | tr '\n' ' '; }
counts() { cat "$1" 2>/dev/null || echo "-"; }
# gdb stops the first publisher when it enters SharedMemTransport::push_discard()
# for one of the dead subscriber's ports (after the send's cleanup_output_ports()),
# marks those ports not ok, as a port watchdog does at any moment, and lets it go
# on. The other publishers are stopped meanwhile, so the first one is the first
# process to test the dead ports for a zombie (the test removes the _el file, so
# only the first can take the recovery path; on a robot that is a matter of luck).
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
cut_off=0; rounds_ok=0
for r in $(seq 1 "$CASES"); do
  rm -f /dev/shm/fast* /dev/shm/sem.fast* /tmp/c1 /tmp/c2 2>/dev/null
  PIDS=()
  for i in 1 2; do "$CC" --ros-args -r __node:=cc$i >/dev/null 2>&1 & PIDS+=($!); sleep 0.3; done
  SEND=()
  for i in $(seq 1 $NS); do
    ros2 topic pub -r 20 "/b$i" std_msgs/msg/String "{data: b$i}" >/dev/null 2>&1 & SEND+=($!); sleep 0.3
  done
  python3 /repro/counter.py /tmp/c1 $NS >/dev/null 2>&1 & V=$!
  sleep 10
  before=$(counts /tmp/c1)
  ports=$(own_ports "$V")
  if [ -z "$ports" ] || [ "$before" = "-" ] || echo "$before" | grep -qw 0; then
    echo "round $r: SETUP victim ports=[$ports] counts=[$before]"
    kill -9 "${PIDS[@]}" "${SEND[@]}" "$V" 2>/dev/null; wait 2>/dev/null; continue
  fi
  kill -STOP "$V"
  flip=""
  case "$LEVER" in
    processing|flip)
      # no watchdog may mark the ports first: the flip below does it inside a send
      python3 /repro/plant.py nowatch $ports >/dev/null
      [ "$LEVER" = processing ] && python3 /repro/plant.py processing $ports >/dev/null
      kill -9 "$V"; wait "$V" 2>/dev/null
      # shellcheck disable=SC2086  # one argument per port
      flip=$(flip_in_send "${SEND[0]}" "${SEND[*]:1}" $ports) ;;
    none) kill -9 "$V"; wait "$V" 2>/dev/null ;;
  esac
  if [ "$flip" = "no flip" ]; then
    echo "round $r: SETUP no send to the dead ports caught"
    kill -9 "${PIDS[@]}" "${SEND[@]}" 2>/dev/null; wait 2>/dev/null; continue
  fi
  rounds_ok=$((rounds_ok + 1))
  sleep 8
  python3 /repro/counter.py /tmp/c2 $NS >/dev/null 2>&1 & V2=$!
  sleep 15
  after=$(counts /tmp/c2)
  wedge=$(python3 /repro/wedge.py "${SEND[@]}" 2>/dev/null | grep '^SELF' | sed 's/^SELF //' | tr '\n' ';')
  # A live publisher the restarted subscriber never hears from is cut off.
  n0=$(echo "$after" | tr ' ' '\n' | grep -c '^0$')
  [ "$after" = "-" ] && n0=$NS
  if [ "$n0" -gt 0 ]; then cut_off=$((cut_off + 1)); v=CUT_OFF; else v=ok; fi
  echo "round $r: $v victim ports [$ports] ${flip:+($flip) }counts before [$before] after restart [$after] self-held port mutexes [${wedge}]"
  kill -9 "${PIDS[@]}" "${SEND[@]}" "$V2" 2>/dev/null; wait 2>/dev/null; sleep 1
done
if [ "$cut_off" -gt 0 ]; then echo "VERDICT: WEDGED ($cut_off of $rounds_ok rounds, lever $LEVER)"
elif [ "$rounds_ok" -eq 0 ]; then echo "VERDICT: SETUP (no round reached the kill)"
else echo "VERDICT: RECOVERED (0 of $rounds_ok rounds, lever $LEVER)"; fi
exit 0
