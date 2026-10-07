#!/bin/bash
# build.sh <distro> [outdir]
# Rebuilds the ROS apt package ros-<distro>-fastrtps (Humble, Jazzy) or
# ros-<distro>-fastdds (Lyrical) from its own source package with every patch
# under patches/<upstream version>/, for the architecture of this host, and
# writes the .deb to <outdir> (default out/<distro>/). The version is the
# stock binary version (distros.env's version field, or the one the pinned base
# image carries) plus "+husarion<N>", so it sorts above the package it
# replaces: v1 was +husarion1 (0001), v2 +husarion4 (0001-0003; +husarion2 and
# +husarion3 were unreleased test builds, and the +husarion2 one is broken, so
# the name is never reused), v3 is +husarion5 (0001-0003, Jazzy on 2.14.7).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
DISTRO=${1:?usage: build.sh <distro> [outdir]}
OUT=$(realpath -m "${2:-$HERE/out/$DISTRO}")
read -r _ PKG DIGEST PIN < <(grep -E "^$DISTRO " "$HERE/distros.env") || { echo "unknown distro $DISTRO" >&2; exit 2; }
SUFFIX=${SUFFIX:-+husarion5}
mkdir -p "$OUT"
docker run --rm -e DISTRO="$DISTRO" -e PKG="$PKG" -e SUFFIX="$SUFFIX" -e PIN="${PIN:-}" -e HOST_UID="$(id -u)" \
  -v "$HERE/patches:/patches:ro" -v "$HERE/scripts:/scripts:ro" -v "$OUT:/out" \
  "ros:$DISTRO-ros-base@$DIGEST" bash /scripts/build-inner.sh
ls -l "$OUT"
