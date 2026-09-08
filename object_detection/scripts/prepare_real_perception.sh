#!/usr/bin/env bash
# Prepare dependencies and keep only our own Zenoh bridge alive. No robot commands.
set -eo pipefail
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then source "$HOME/ros2_ws/install/setup.bash"; fi
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ZENOH_DIR="${ZENOH_DIR:-$HOME/ros2_ws/src/zenoh-pointcloud}"
CONFIG="$ZENOH_DIR/config/zenoh-plugin-dds/rosject.json5"
TOPIC="/camera/depth/color/points"
ENDPOINT=""
if [[ "${1:-}" == "--help" ]]; then
  echo "Usage: $0 [--endpoint tcp/ADDRESS:7447]"
  echo 'Checks/installs PCL and Zenoh when missing; verifies real PointCloud2 reception.'
  echo 'Uses current Zenoh config unless --endpoint is supplied; Ctrl+C stops our bridge.'
  exit 0
fi
if [[ $# -gt 0 ]]; then
  [[ $# -eq 2 && "$1" == "--endpoint" && "$2" == tcp/* ]] || { echo 'Invalid arguments; use --help.'; exit 2; }
  ENDPOINT="$2"
fi
bash "$SCRIPT_DIR/check_pcl.sh"
probe() {
  timeout 15 /usr/bin/python3 "$SCRIPT_DIR/probe_pointcloud.py" "$TOPIC"
}
if [[ -z "$ENDPOINT" ]] && probe; then
  echo 'READY: PCL works and real PointCloud2 is being received; no bridge started.'
  exit 0
fi
if pgrep -f '^([^ ]*/)?zenoh-bridge-ros2dds([[:space:]]|$)' >/dev/null; then
  echo 'An existing Zenoh bridge is running. Stop it in its terminal before retrying; this script will not replace another process.'
  exit 1
fi
[[ -f "$CONFIG" ]] || { echo "Missing configuration: $CONFIG"; exit 1; }
if [[ -n "$ENDPOINT" ]]; then
  /usr/bin/python3 - "$CONFIG" "$ENDPOINT" <<'PY'
import json, pathlib, re, shutil, sys, time
path = pathlib.Path(sys.argv[1])
text = path.read_text()
pattern = r'(\bconnect\s*:\s*\{\s*endpoints\s*:\s*\[\s*)"[^"]+"'
if len(re.findall(pattern, text)) != 1:
    raise SystemExit('Expected one connect/endpoints entry; configuration unchanged.')
updated = re.sub(pattern, lambda m: m[1] + json.dumps(sys.argv[2]), text)
if updated != text:
    backup = str(path) + '.backup.' + str(time.time_ns())
    shutil.copy2(path, backup)
    path.write_text(updated)
    print('Endpoint updated; backup:', backup)
PY
fi
if ! command -v zenoh-bridge-ros2dds >/dev/null; then
  bash "$SCRIPT_DIR/install_zenoh_bridge.sh"
fi
LOG_DIR="$HOME/.ros/real_perception"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/zenoh-$(date +%Y%m%d-%H%M%S)-$$.log"
BRIDGE_PID=""
cleanup() {
  if [[ -n "$BRIDGE_PID" ]] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
    kill -TERM "$BRIDGE_PID" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "$BRIDGE_PID" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL "$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
zenoh-bridge-ros2dds -c "$CONFIG" >"$LOG" 2>&1 &
BRIDGE_PID=$!
echo "Bridge PID=$BRIDGE_PID; log: $LOG"
for attempt in 1 2 3; do
  if probe && kill -0 "$BRIDGE_PID" 2>/dev/null; then
    echo 'READY: received PointCloud2. Keep this terminal open; Ctrl+C stops this bridge.'
    wait "$BRIDGE_PID"
    exit $?
  fi
  kill -0 "$BRIDGE_PID" 2>/dev/null || break
done
echo 'No point cloud received. Check the robot reservation, current endpoint and network; reinstalling does not repair a remote connection.'
tail -n 35 "$LOG"
exit 1
