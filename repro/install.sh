# Sourced by the inner reproducers before the distro's setup script (no set -u
# there). Inputs: DISTRO, PKG (fastrtps or fastdds), PIN (distros.env's source
# version field, may be empty), /pkg/*.deb (the package under test), /mix/*.deb
# (optional, a second package for the uid-10001 participants).
#
# With /pkg/*.deb, that package is installed. Without one, the stock package is
# tested: the pinned version from the ROS repository when distros.env pins one
# the base image does not carry (the repository can be ahead of every base
# image), else the base image's own.
#
# With /mix/*.deb, that package is unpacked to /opt/mix and MIX_LD names its
# library directory: the reproducers that run participants as uid 10001 (an
# airlock half joining a root robot world) put it first on their
# LD_LIBRARY_PATH, so one world mixes two Fast DDS builds, as a robot does when
# its images were built at different times. mix_libs() reports which build each
# given process mapped.
name="ros-$DISTRO-$PKG"
MIX_LD=""
if ls /pkg/*.deb >/dev/null 2>&1; then
  dpkg -i /pkg/*.deb >/dev/null 2>&1 || { echo "dpkg -i failed"; return 1; }
  echo "installed: $(dpkg-query -W -f='${Package} ${Version}' "$(dpkg-deb -f /pkg/*.deb Package)")"
else
  # PIN is the source version; the binary adds a per-architecture build stamp.
  if [ -n "${PIN:-}" ] && ! dpkg-query -W -f='${Version}' "$name" | grep -q "^$PIN\(\.\|$\)"; then
    apt-get update -qq >/dev/null 2>&1
    pin_bin=$(apt-cache madison "$name" | awk -v p="$PIN" '{v=$3} v==p || index(v, p ".")==1 {print v; exit}')
    { [ -n "$pin_bin" ] && apt-get install -y -qq --no-install-recommends "$name=$pin_bin"; } >/dev/null 2>&1 \
      || { echo "cannot install the pinned stock $name $PIN"; return 1; }
  fi
  echo "stock: $name $(dpkg-query -W -f='${Version}' "$name")"
fi
if ls /mix/*.deb >/dev/null 2>&1; then
  # An image built against the mix package also has the ROS packages that link
  # Fast DDS rebuilt against it (rmw_fastrtps, the typesupports): unpack the
  # repository's current versions of the installed ones that link it directly,
  # then the mix package over them.
  rebuilt=""
  if [ "$(dpkg-deb -f /mix/*.deb Version | cut -d- -f1)" != "$(dpkg-query -W -f='${Version}' "$name" | cut -d- -f1)" ]; then
    apt-get update -qq >/dev/null 2>&1
    mkdir -p /tmp/mixdl
    for p in $(apt-cache rdepends --installed "$name" | sed -n 's/^  *\(ros-[^ ]*\)$/\1/p' | sort -u); do
      c=$(apt-cache policy "$p" | sed -n 's/^  Candidate: //p'); i=$(dpkg-query -W -f='${Version}' "$p")
      [ -n "$c" ] && [ "$c" != "$i" ] || continue
      (cd /tmp/mixdl && apt-get download -qq "$p=$c" >/dev/null 2>&1) || { echo "cannot download $p=$c"; return 1; }
      rebuilt+="$p "
    done
    for d in /tmp/mixdl/*.deb; do [ -e "$d" ] && dpkg-deb -x "$d" /opt/mix; done
  fi
  dpkg-deb -x /mix/*.deb /opt/mix || { echo "cannot unpack the mix package"; return 1; }
  MIX_LD=/opt/mix/opt/ros/$DISTRO/lib
  echo "mixed: uid-10001 participants load $(dpkg-deb -f /mix/*.deb Package) $(dpkg-deb -f /mix/*.deb Version) from $MIX_LD, with the repository's ${rebuilt:-(none)}"
fi
# mix_libs <pid>...: "<main> main, <mix> mix, <n> unread" by the Fast DDS library
# each process mapped (a uid-10001 process's maps need SYS_PTRACE)
mix_libs() {
  local p m main=0 mix=0 unread=0
  for p in "$@"; do
    m=$(cat "/proc/$p/maps" 2>/dev/null) || m=""
    if grep -q " /opt/mix/.*libfast\(rtps\|dds\)\.so" <<< "$m"; then mix=$((mix + 1))
    elif grep -q "libfast\(rtps\|dds\)\.so" <<< "$m"; then main=$((main + 1))
    else unread=$((unread + 1)); fi
  done
  echo "$main main, $mix mix, $unread unread"
}
