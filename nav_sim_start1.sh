#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_WS="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ ! -d "$SIM_WS/dexe_mobile_application" || ! -d "$SIM_WS/dexe_mobile_nav" ]]; then
  echo "Error: expected dexe_mobile_application and dexe_mobile_nav under: $SIM_WS" >&2
  exit 1
fi

cd "$SIM_WS"

pkill gzserver || true
pkill gzclient || true

set +u
source /opt/ros/humble/setup.bash
source "$SIM_WS/install/setup.bash"
source /usr/share/gazebo/setup.sh
set -u

export GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MODEL_PATH="$SIM_WS/install/buddi_gazebo/share:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_MODEL_PATH="$SIM_WS/install/aws_robomaker_small_warehouse_world/share/aws_robomaker_small_warehouse_world/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-}"
export LIBGL_ALWAYS_SOFTWARE=1

cleanup() {
  if [[ -n "${TF_PID:-}" ]]; then
    kill "${TF_PID}" 2>/dev/null || true
  fi
  if [[ -n "${GAZEBO_PID:-}" ]]; then
    kill "${GAZEBO_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ros2 launch buddy_nav2 buddy_nav2_gazebo.launch.py &
GAZEBO_PID=$!

sleep "${SIM_GAZEBO_WAIT_SECONDS:-5}"

ros2 run tf2_ros static_transform_publisher \
  --x -1.875 \
  --y -1.825 \
  --z 0 \
  --roll 0 \
  --pitch 0 \
  --yaw 0 \
  --frame-id map \
  --child-frame-id odom &
TF_PID=$!

wait
