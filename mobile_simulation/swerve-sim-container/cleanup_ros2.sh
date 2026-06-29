#!/bin/bash
# 清理 swerve 仿真 + Nav2 相关后台进程。
# 注意：若用 Docker 跑仿真 (make launch)，必须先 docker stop，主机 pkill 杀不到容器内进程。
# 不用 set -u：source /opt/ros/humble/setup.bash 会引用未设置的 AMENT_TRACE_SETUP_FILES。
#
# CLEANUP_APPLICATION=1  同时清理 dexforce 可视化 / 末端仿真栈（ee_*_sim_server、visual_ros2_node 等）。
# 默认只清理仿真+Nav2；验证时也仅检查仿真相关节点，其它栈节点仅提示、不算失败。

NAV_WS="${NAV_WS:-/home/dex/dexe_mobile_nav}"
CLEANUP_APPLICATION="${CLEANUP_APPLICATION:-0}"

echo "=========================================="
echo "      ROS2 仿真环境清理脚本"
echo "=========================================="

echo ""
echo "[1/9] 停止 Docker 仿真容器 (如有)..."
if command -v docker >/dev/null 2>&1; then
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx 'swerve-sim-container'; then
        docker stop swerve-sim-container || true
        echo "  已 stop swerve-sim-container"
    fi
fi

echo "[2/9] 停止 launch 父进程..."
pkill -9 -f "ros2 launch swerve_robot_gazebo" 2>/dev/null || true
pkill -9 -f "ros2 launch nav2_bringup" 2>/dev/null || true
pkill -9 -f "ros2 launch w1_nav2" 2>/dev/null || true
pkill -9 -f "ros2 launch.*slam" 2>/dev/null || true

echo "[3/9] 停止 Gazebo..."
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true

echo "[4/9] 停止 Nav2 / 控制器..."
pkill -9 -f component_container_isolated 2>/dev/null || true
pkill -9 -f component_container 2>/dev/null || true
pkill -9 -f "/controller_manager" 2>/dev/null || true
pkill -9 -f "controller_manager/spawner" 2>/dev/null || true
pkill -9 -f "executable=spawner" 2>/dev/null || true
pkill -9 -f spawn_entity.py 2>/dev/null || true
pkill -9 -f lifecycle_manager 2>/dev/null || true

echo "[5/9] 停止控制与 TF 节点..."
pkill -9 -f wheel_cmd_publisher 2>/dev/null || true
pkill -9 -f wheel_odometry 2>/dev/null || true
pkill -9 -f swerve_commander 2>/dev/null || true
pkill -9 -f swerve_odometer 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f static_transform_publisher 2>/dev/null || true
pkill -9 -f map_to_odom_static_tf 2>/dev/null || true

echo "[6/9] 停止 RViz 与 Teleop..."
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f teleop_twist 2>/dev/null || true

if [[ "$CLEANUP_APPLICATION" == "1" ]]; then
    echo "[6b/9] 停止 dexforce 可视化 / 末端仿真栈..."
    pkill -9 -f "ros2 launch dexforce_bringup" 2>/dev/null || true
    pkill -9 -f dexhand_joint_converter 2>/dev/null || true
    pkill -9 -f general_joint_converter 2>/dev/null || true
    pkill -9 -f ee_left_sim_server 2>/dev/null || true
    pkill -9 -f ee_right_sim_server 2>/dev/null || true
    pkill -9 -f ee_sim_server_launcher 2>/dev/null || true
    pkill -9 -f visual_ros2 2>/dev/null || true
fi

echo "[7/9] 停止其它 ros2 launch/run..."
pkill -9 -f "ros2 launch swerve_robot_gazebo" 2>/dev/null || true
pkill -9 -f "ros2 launch nav2_bringup" 2>/dev/null || true
pkill -9 -f "ros2 launch w1_nav2" 2>/dev/null || true
pkill -9 -f "ros2 launch.*slam" 2>/dev/null || true
pkill -9 -f "ros2 run.*swerve" 2>/dev/null || true
pkill -9 -f "ros2 run.*nav2" 2>/dev/null || true

echo "[8/9] 停止 ros2 daemon (清除幽灵节点缓存)..."
if command -v ros2 >/dev/null 2>&1; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    ros2 daemon stop 2>/dev/null || true
fi

echo "[9/9] 等待进程退出..."
sleep 2

echo ""
echo "================ 验证 ================"

# 只匹配真实 ROS/Gazebo 可执行进程，避免 sim.launch 误匹配普通 shell 命令行
collect_left_processes() {
    pgrep -af \
        'gzserver|gzclient|spawn_entity\.py|component_container|wheel_cmd_publisher|wheel_odometry|robot_state_publisher|static_transform_publisher|/opt/ros/.*/lib/.*/spawner|controller_manager' \
        2>/dev/null \
        | grep -Ev 'grep|cleanup_ros2\.sh' || true
}

LEFT_PROCESS="$(collect_left_processes)"

# 仿真/Nav2 相关节点（清理失败才报）
SIM_NODE_PATTERN='controller_manager|gazebo|gz_|nav2|lifecycle_manager|map_server|planner_server|bt_navigator|behavior_server|velocity_smoother|amcl|costmap|wheel_|swerve_|robot_state_publisher|static_transform|map_to_odom|spawn_entity|component_container|smoother_server|waypoint_follower|dock|collision_monitor|route_server'
# 其它栈（dexforce 可视化 / 末端仿真），不算仿真清理失败
OTHER_NODE_PATTERN='ee_.*sim|visual_ros2|dexhand_joint_converter|general_joint_converter|brainco_hands|system_monitor'

filter_sim_nodes() {
    grep -E "/(${SIM_NODE_PATTERN})" || true
}

filter_other_nodes() {
    grep -E "/(${OTHER_NODE_PATTERN})" || true
}

LEFT_SIM_NODE=""
LEFT_OTHER_NODE=""
ALL_NODES=""
if command -v ros2 >/dev/null 2>&1; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    if [[ -f "${NAV_WS}/install/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "${NAV_WS}/install/setup.bash"
    fi
    export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
    # daemon 已 stop；这里 list 会拉起新 daemon，读到的是当前 DDS 图
    ALL_NODES="$(ros2 node list 2>/dev/null | grep -v '^$' || true)"
    LEFT_SIM_NODE="$(printf '%s\n' "$ALL_NODES" | filter_sim_nodes)"
    LEFT_OTHER_NODE="$(printf '%s\n' "$ALL_NODES" | filter_other_nodes)"
fi

if [[ -z "$LEFT_PROCESS" && -z "$LEFT_SIM_NODE" ]]; then
    echo ""
    echo "✅ 仿真/Nav2 已全部清理完成"
    if [[ -n "$LEFT_OTHER_NODE" ]]; then
        echo ""
        if [[ "$CLEANUP_APPLICATION" == "1" ]]; then
            echo "⚠ 以下其它栈节点仍在运行（pkill 未命中，可能在其它终端/Docker/系统服务）："
        else
            echo "ℹ 以下节点来自其它栈（非 swerve 仿真），可忽略；若要一并关闭："
            echo "  CLEANUP_APPLICATION=1 ./cleanup_ros2.sh"
        fi
        echo ""
        echo "【其它栈 ROS 节点（未计入失败）】"
        echo "$LEFT_OTHER_NODE"
        if [[ "$CLEANUP_APPLICATION" == "1" ]]; then
            echo ""
            echo "  手动排查: pgrep -af 'ee_.*sim|visual_ros2|dexforce_bringup'"
        fi
    fi
    echo "=========================================="
    exit 0
fi

echo ""
echo "⚠ 仿真/Nav2 仍有残余（重复运行通常无法消除，请看下方原因）"
echo ""

if [[ -n "$LEFT_PROCESS" ]]; then
    echo "【残余进程 — 请手动 kill -9 <pid> 或补充 pkill 规则】"
    echo "$LEFT_PROCESS"
    echo ""
fi

if [[ -n "$LEFT_SIM_NODE" ]]; then
    echo "【残余仿真/Nav2 ROS 节点】"
    echo "$LEFT_SIM_NODE"
    echo ""
fi

if [[ -n "$LEFT_OTHER_NODE" ]]; then
    echo "【其它栈 ROS 节点（非仿真，daemon stop 无法消除真实进程）】"
    echo "$LEFT_OTHER_NODE"
    echo ""
    echo "  这些通常来自 dexforce_bringup / 末端仿真，与 swerve 仿真无关。"
    echo "  若要一并清理: CLEANUP_APPLICATION=1 ./cleanup_ros2.sh"
    echo "  或手动查进程: pgrep -af 'ee_.*sim|visual_ros2|dexforce_bringup'"
    echo ""
fi

if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx 'swerve-sim-container'; then
    echo "【Docker】swerve-sim-container 仍在运行，主机 pkill 无效："
    echo "  docker stop swerve-sim-container"
    echo ""
fi

echo "=========================================="
exit 1
