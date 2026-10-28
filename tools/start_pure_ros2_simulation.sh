#!/usr/bin/env bash
# ============================================================================
# DEXE Mobile Navigation - 纯原生 ROS 2 插件化闭环仿真一键启动脚本 (基于 w1_diff_sim.urdf)
# 架构组成:
#   1. w1_diff_kinematic_emulator : 100Hz 差速轮系运动学递推 (/feedback/odom + /joint_states + TF)
#                                   严格采用 w1_diff_sim.urdf 参数 (b=0.5078m, D=0.126m, a_max=1.0m/s^2)
#   2. robot_state_publisher      : 加载 w1_diff_sim.urdf 广播全车连杆与传感器 TF 树
#   3. controller_server          : 官方 Nav2 控制服务器，内置避障状态机 (10Hz)，
#                                   动态加载 PathPruningLocalPlanner / FrenetLocalPlanner / LexLocalPlanner
#                                   + SpeedPlanner (S曲线速度剖面) + RPPController (25Hz 底层跟踪)
#   4. lifecycle_manager          : 管理 controller_server 生命周期，自动切换至 Active
#   5. rosbridge_websocket        : ROS 2 WebSocket 网关 (ws://localhost:9090)
#   6. route_planner_native_server: 静态 Web 与拓扑路网规划服务 (http://localhost:8765)
# ============================================================================

set -e

WORKSPACE_DIR="/home/dex/Desktop/work/kuawei/dexe_mobile_nav"
URDF_FILE="${WORKSPACE_DIR}/mobile_simulation/diff-sim-container/src/w1_diff_gazebo/urdf/w1_diff_sim.urdf"
cd "${WORKSPACE_DIR}"

# 确保使用系统原生 Python 3.10，避免 miniconda 路径污染 ROS 2 环境
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v 'miniconda' | paste -sd:)
export PATH=/usr/bin:/usr/local/bin:$PATH
unset PYTHONPATH

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

LOG_DIR="${WORKSPACE_DIR}/log/pure_ros2_sim"
mkdir -p "${LOG_DIR}"

PARAMS_FILE="${WORKSPACE_DIR}/mobile_navigation/nav2_bringup/params/w1_nav2_params_diff.yaml"

echo "🧹 清理旧的仿真节点进程..."
pkill -f "w1_diff_kinematic_emulator" 2>/dev/null || true
pkill -f "controller_server" 2>/dev/null || true
pkill -f "lifecycle_manager_pure_sim" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
pkill -f "rosbridge_websocket" 2>/dev/null || true
pkill -f "route_planner_action_bridge.py" 2>/dev/null || true
sleep 0.5

echo "🚀 [1/4] 启动 ROS 2 纯仿真集成 Launch (w1_diff_pure_sim.launch.py)..."
nohup ros2 launch w1_diff_gazebo w1_diff_pure_sim.launch.py \
  params_file:="${PARAMS_FILE}" \
  use_sim_time:=false \
  autostart:=true \
  enable_rosbridge:=true \
  > "${LOG_DIR}/pure_sim_launch.log" 2>&1 &
LAUNCH_PID=$!

if ! pgrep -f "route_planner_native_server.py" > /dev/null; then
  echo "🖥️ [2/4] 启动 Web 前端与路网服务 (http://localhost:8765)..."
  nohup /usr/bin/python3 "${WORKSPACE_DIR}/tools/route_planner_native_server.py" --port 8765 \
    > "${LOG_DIR}/web_server.log" 2>&1 &
else
  echo "🖥️ [2/4] Web 前端服务已在运行 (http://localhost:8765)"
fi

sleep 2.0

echo ""
echo "============================================================================"
echo "✅ 纯原生 ROS 2 插件化仿真系统 (w1_diff_sim.urdf) 已全部就绪！"
echo "  • 核心 Launch 进程 PID   : ${LAUNCH_PID} (w1_diff_pure_sim.launch.py)"
echo "  • 运动学递推参数         : 轮距 0.5078m, 轮径 0.126m (来自 w1_diff_sim.urdf)"
echo "  • 控制层与状态机         : 官方 ControllerServer (内置 10Hz 避障状态机与可选回退)"
echo "  • 局部规划插件池         : PathPruningLocalPlanner / FrenetLocalPlanner / LexLocalPlanner"
echo "  • WebSocket 网关         : ws://localhost:9090"
echo "  • 前端可视化仿真交互页面 : http://localhost:8765/route_grid_planner_visualizer.html"
echo "  • 运行日志目录           : ${LOG_DIR}"
echo "============================================================================"

if [[ "$1" == "--daemon-wait" ]]; then
  wait
fi
