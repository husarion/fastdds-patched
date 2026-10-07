#!/bin/bash
# Runs inside ros:<distro>-ros-base (build.sh). Inputs: DISTRO, PKG, SUFFIX,
# PIN (optional: the stock source version to patch), HOST_UID, /patches,
# output to /out.
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
name="ros-$DISTRO-$PKG"
src=$(grep -l packages.ros.org /etc/apt/sources.list.d/* | head -1)
# The ROS repo serves its source packages too: enable deb-src for it.
if grep -q '^Types:' "$src"; then sed -i 's/^Types: deb$/Types: deb deb-src/' "$src"; else sed -i 's/^deb \(.*\)/deb \1\ndeb-src \1/' "$src"; fi
apt-get update -qq
apt-get install -y -qq --no-install-recommends dpkg-dev fakeroot >/dev/null
# The repository can be ahead of the base image: install the pinned stock
# version first, so the build patches exactly that one.
# PIN is the source version; the binary adds a per-architecture build stamp.
if [ -n "${PIN:-}" ]; then
  pin_bin=$(apt-cache madison "$name" | awk -v p="$PIN" '{v=$3} v==p || index(v, p ".")==1 {print v; exit}')
  [ -n "$pin_bin" ] || { echo "the repository has no $name $PIN" >&2; exit 1; }
  apt-get install -y -qq --no-install-recommends "$name=$pin_bin" >/dev/null
fi
bin_ver=$(dpkg-query -W -f='${Version}' "$name")
up_ver=${bin_ver%%-*}
patchdir=/patches/$up_ver
[ -d "$patchdir" ] || { echo "no patches/$up_ver for $name $bin_ver: add the patch for this upstream version" >&2; exit 1; }
src_ver=$(apt-cache showsrc "$name" | awk '/^Version:/{print $2}' | sort -V | tail -1)
[ "${bin_ver#"$src_ver"}" != "$bin_ver" ] || { echo "source $src_ver does not match the installed $bin_ver" >&2; exit 1; }
apt-get build-dep -y -qq "$name" >/dev/null
mkdir -p /b && cd /b
apt-get source -qq "$name" >/dev/null
cd "$(find /b -mindepth 1 -maxdepth 1 -type d | head -1)"
got=$(dpkg-parsechangelog -S Version)
[ "$got" = "$src_ver" ] || { echo "apt-get source gave $got, expected $src_ver" >&2; exit 1; }
for p in "$patchdir"/*.patch; do echo "applying $(basename "$p")"; patch -p1 --forward < "$p"; done
new_ver="$bin_ver$SUFFIX"
{ printf '%s (%s) %s; urgency=high\n\n' "$name" "$new_ver" "$(lsb_release -cs)"
  for p in "$patchdir"/*.patch; do printf '  * %s\n' "$(sed -n 's/^Subject: \(\[PATCH[^]]*\] \)\{0,1\}//p' "$p" | head -1)"; done
  printf '\n -- Husarion <support@husarion.com>  %s\n\n' "$(date -R)"
  cat debian/changelog; } > /tmp/changelog && mv /tmp/changelog debian/changelog
DEB_BUILD_OPTIONS="nocheck parallel=$(nproc)" dpkg-buildpackage -b -uc -us >/b/build.log 2>&1 || { tail -40 /b/build.log; exit 1; }
cp /b/"${name}"_*.deb /out/
chown -R "$HOST_UID" /out
echo "built $name $new_ver"
