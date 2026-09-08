#!/usr/bin/env bash
# Reuse the course installer only when the bridge is missing.
set -euo pipefail
if command -v zenoh-bridge-ros2dds >/dev/null; then
  echo 'Zenoh bridge already available; no installation needed.'
  exit 0
fi
ZENOH_DIR="${ZENOH_DIR:-$HOME/ros2_ws/src/zenoh-pointcloud}"
INSTALLER="$ZENOH_DIR/install_zenoh.sh"
[[ -f "$INSTALLER" ]] || { echo "Missing course installer: $INSTALLER"; exit 1; }
echo "Zenoh missing; running course installer: $INSTALLER"
# Its package post-install may fail on systemd in the cloud container.
# Check the resulting executable rather than treating every apt error as success.
if ! (cd "$ZENOH_DIR" && bash ./install_zenoh.sh); then
  echo 'Course installer reported an error; checking whether the bridge was installed.'
fi
if ! command -v zenoh-bridge-ros2dds >/dev/null; then
  echo 'Zenoh is still unavailable. Inspect the installer output; point-cloud startup stopped.'
  exit 1
fi
zenoh-bridge-ros2dds --version