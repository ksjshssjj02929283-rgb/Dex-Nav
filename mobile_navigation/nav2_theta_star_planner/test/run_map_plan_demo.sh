#!/usr/bin/env bash
# Offline map planning demo — no Gazebo.
#
# Usage:
#   ./run_map_plan_demo.sh /path/to/map.yaml
#   ./run_map_plan_demo.sh /path/to/map.yaml --no-build-nodemap
#   ./run_map_plan_demo.sh /path/to/map.yaml --rebuild
#   ./run_map_plan_demo.sh /path/to/map.yaml params_file:=/path/to/custom_params.yaml
#
# RViz:
#   1. "2D Pose Estimate" → start (default origin if skipped)
#   2. "2D Nav Goal"      → plan (shown on /plan + planner_server log)

# set -u 会触发 ROS setup.bash 里 AMENT_TRACE_SETUP_FILES 等未定义变量报错
set -eo pipefail

# Run from the workspace root, or set NAV_WS explicitly.
NAV_WS="${NAV_WS:-$(pwd)}"
BUILD_NODEMAP=1
REBUILD_PLANNER=0

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <map.yaml> [extra ros2 launch args...]" >&2
  echo "Example: $0 /home/dex/maps/my_map.yaml" >&2
  exit 1
fi

MAP="$(readlink -f "$1")"
shift

for arg in "$@"; do
  if [[ "$arg" == "--no-build-nodemap" ]]; then
    BUILD_NODEMAP=0
  elif [[ "$arg" == "--rebuild" ]]; then
    REBUILD_PLANNER=1
  fi
done

LAUNCH_ARGS=()
for arg in "$@"; do
  if [[ "$arg" != "--no-build-nodemap" && "$arg" != "--rebuild" ]]; then
    LAUNCH_ARGS+=("$arg")
  fi
done

planner_so_has_l1_spline() {
  local so="$1"
  grep -aq 'L1 spline path' "$so" 2>/dev/null && return 0
  if command -v strings >/dev/null 2>&1; then
    strings "$so" 2>/dev/null | grep -q 'L1 spline path'
    return $?
  fi
  return 1
}

if [[ ! -f "$MAP" ]]; then
  echo "Map not found: $MAP" >&2
  exit 1
fi

MAP_DIR="$(dirname "$MAP")"
PRE="${MAP_DIR}/preprocessed/nodemap_global.yaml"

source /opt/ros/humble/setup.bash

BUILD_CMD=(
  colcon build
  --packages-select nav2_map_server nav2_theta_star_planner
  --symlink-install
  --allow-overriding nav2_map_server nav2_theta_star_planner
)

if [[ "$REBUILD_PLANNER" -eq 1 ]]; then
  echo "==> Rebuilding nav2_theta_star_planner in ${NAV_WS}"
  (cd "${NAV_WS}" && "${BUILD_CMD[@]}")
fi

ROOT_MAP_SERVER="${NAV_WS}/install/nav2_map_server"
ROOT_PLANNER="${NAV_WS}/install/nav2_theta_star_planner"
MAP_SERVER_SETUP="${ROOT_MAP_SERVER}/share/nav2_map_server/local_setup.bash"
PLANNER_SETUP="${ROOT_PLANNER}/share/nav2_theta_star_planner/local_setup.bash"

# Source only the two packages used by this demo. The workspace-wide setup may
# contain stale package entries from unrelated partial builds.
if [[ ! -f "${MAP_SERVER_SETUP}" || ! -f "${PLANNER_SETUP}" ]]; then
  echo "ERROR: NodeMap demo packages are not built in ${NAV_WS}/install." >&2
  echo "       cd ${NAV_WS} && ${BUILD_CMD[*]}" >&2
  exit 1
fi
source "${MAP_SERVER_SETUP}"
source "${PLANNER_SETUP}"

PLANNER_SO="${ROOT_PLANNER}/lib/libnav2_theta_star_planner.so"
PLANNER_SO_REAL="$(readlink -f "${PLANNER_SO}" 2>/dev/null || true)"
if [[ ! -f "${PLANNER_SO}" ]]; then
  echo "ERROR: planner not built in root install: ${PLANNER_SO}" >&2
  echo "       cd ${NAV_WS} && ${BUILD_CMD[*]}" >&2
  exit 1
fi
if ! planner_so_has_l1_spline "${PLANNER_SO}"; then
  echo "ERROR: root planner .so is outdated (missing L1 spline)." >&2
  echo "       ${PLANNER_SO}" >&2
  if [[ -n "${PLANNER_SO_REAL}" ]]; then
    echo "       -> ${PLANNER_SO_REAL} ($(stat -c '%y %s bytes' "${PLANNER_SO_REAL}" 2>/dev/null || echo 'stat failed'))" >&2
  fi
  echo "       Rebuild, then re-source and restart launch (not hot-reload):" >&2
  echo "         cd ${NAV_WS}" >&2
  echo "         source /opt/ros/humble/setup.bash" >&2
  echo "         ${BUILD_CMD[*]}" >&2
  echo "         source install/setup.bash" >&2
  echo "       Or: $0 --rebuild ${MAP} ${LAUNCH_ARGS[*]}" >&2
  exit 1
fi

RESOLVED_PREFIX="$(ros2 pkg prefix nav2_theta_star_planner 2>/dev/null || true)"
if [[ "${RESOLVED_PREFIX}" == *"/mobile_navigation/install"* ]]; then
  echo "WARNING: ros2 still resolves planner to stale underlay: ${RESOLVED_PREFIX}" >&2
  echo "         Using root install explicitly: ${ROOT_PLANNER}" >&2
fi
echo "==> nav2_theta_star_planner (root): ${ROOT_PLANNER}"

PKG_SHARE="${ROOT_PLANNER}/share/nav2_theta_star_planner"
LAUNCH_FILE="${PKG_SHARE}/test/map_plan_demo.launch.py"

if [[ ! -f "$LAUNCH_FILE" ]]; then
  echo "Launch file not installed. Build and source workspace first:" >&2
  echo "  cd ${NAV_WS} && ${BUILD_CMD[*]}" >&2
  exit 1
fi

if [[ "$BUILD_NODEMAP" -eq 1 && ! -f "$PRE" ]]; then
  echo "==> Building nodemap (preprocessed/) for: $MAP"
  ros2 run nav2_map_server map_preprocess --map-yaml "$MAP"
  echo "==> Done: $PRE"
elif [[ ! -f "$PRE" ]]; then
  echo "WARNING: No nodemap at $PRE — use_node_map planning will fall back to grid Theta*" >&2
  echo "         Run: ros2 run nav2_map_server map_preprocess --map-yaml $MAP" >&2
fi

echo "==> Launching offline plan demo (map=$MAP)"
exec ros2 launch "$LAUNCH_FILE" "map:=$MAP" "${LAUNCH_ARGS[@]}"
