#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_WS="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ ! -d "$SIM_WS/dexe_mobile_application" || ! -d "$SIM_WS/dexe_mobile_nav" ]]; then
  echo "Error: expected dexe_mobile_application and dexe_mobile_nav under: $SIM_WS" >&2
  exit 1
fi

cd "$SIM_WS"
mkdir -p "$SIM_WS/ros_logs"

set +u
source /opt/ros/humble/setup.bash
source "$SIM_WS/install/setup.bash"
set -u

export LIBGL_ALWAYS_SOFTWARE=1

cleanup() {
  if [[ -n "${CMD_VEL_MUX_PID:-}" ]]; then
    kill "${CMD_VEL_MUX_PID}" 2>/dev/null || true
  fi
  if [[ -n "${RVIZ_PID:-}" ]]; then
    kill "${RVIZ_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ros2 launch cmd_vel_mux cmd_vel_mux-launch.py \
  > "$SIM_WS/ros_logs/cmd_vel_mux.log" 2>&1 &
CMD_VEL_MUX_PID=$!

(
  sleep "${SIM_NAV2_WAIT_SECONDS:-8}"
  ros2 launch nav2_bringup rviz_launch.py \
    use_sim_time:=true \
    rviz_config:="$SIM_WS/install/buddy_nav2/share/buddy_nav2/rviz/b1_simulation_v2.rviz" \
    > "$SIM_WS/ros_logs/rviz.log" 2>&1
) &
RVIZ_PID=$!

ros2 launch nav2_bringup bringup_launch.py \
  slam:=False \
  map:="$SIM_WS/install/aws_robomaker_small_warehouse_world/share/aws_robomaker_small_warehouse_world/maps/005/simulation_mapping_v1_keepout.yaml" \
  use_sim_time:=true \
  params_file:="$SIM_WS/install/nav2_bringup/share/nav2_bringup/params/w1_nav2_params.yaml" \
  autostart:=True \
  use_composition:=False \
  2>&1 | tee "$SIM_WS/ros_logs/nav.log"
