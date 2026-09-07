#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"

check_pcl() {
  "$PYTHON_BIN" - <<'PY'
import pcl
cloud = pcl.PointCloud()
required = (
    "make_segmenter",
    "make_kdtree",
    "make_EuclideanClusterExtraction",
)
missing = [name for name in required if not hasattr(cloud, name)]
if missing:
    raise RuntimeError("missing PCL Python API: " + ", ".join(missing))
print(f"Python PCL OK: {pcl.__file__}")
PY
}

if check_pcl; then
  echo "python3-pcl is already installed; no installation needed."
  exit 0
fi

echo "Python PCL is unavailable for ${PYTHON_BIN}; installing python3-pcl now."
if [[ "${EUID}" -eq 0 ]]; then
  apt-get update
  DEBIAN_FRONTEND=noninteractive apt-get install -y python3-pcl
else
  sudo apt-get update
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y python3-pcl
fi

check_pcl
echo "PCL dependency check passed."
