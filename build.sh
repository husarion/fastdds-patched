#!/bin/bash
# build.sh <distro> [outdir]
# Rebuilds the ROS apt package ros-<distro>-fastrtps (Humble, Jazzy) or
# ros-<distro>-fastdds (Lyrical) from its own source package with every patch
# under patches/<upstream version>/, for the architecture of this host, and
# writes the .deb to <outdir> (default out/<distro>/). The version is the
# installed binary version of the pinned base image plus "+husarion1", so it
# sorts above the package it replaces.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
DISTRO=${1:?usage: build.sh <distro> [outdir]}
OUT=$(realpath -m "${2:-$HERE/out/$DISTRO}")
read -r _ PKG DIGEST < <(grep -E "^$DISTRO " "$HERE/distros.env") || { echo "unknown distro $DISTRO" >&2; exit 2; }
SUFFIX=${SUFFIX:-+husarion1}
mkdir -p "$OUT"
docker run --rm -e DISTRO="$DISTRO" -e PKG="$PKG" -e SUFFIX="$SUFFIX" -e HOST_UID="$(id -u)" \
  -v "$HERE/patches:/patches:ro" -v "$HERE/scripts:/scripts:ro" -v "$OUT:/out" \
  "ros:$DISTRO-ros-base@$DIGEST" bash /scripts/build-inner.sh
ls -l "$OUT"
