#!/bin/bash

echo "=========================================="
echo "      ROS2 仿真环境清理脚本"
echo "=========================================="

echo ""
echo "[1/8] 停止 launch 父进程..."
pkill -9 -f "ros2 launch swerve_robot_gazebo"
pkill -9 -f "ros2 launch nav2_bringup"
pkill -9 -f "ros2 launch.*slam"

echo "[2/8] 停止 Gazebo..."
pkill -9 -f gzserver
pkill -9 -f gzclient

echo "[3/8] 停止 Nav2 Container..."
pkill -9 -f component_container_isolated

echo "[4/8] 停止控制与TF节点..."
pkill -9 -f wheel_cmd_publisher
pkill -9 -f wheel_odometry
pkill -9 -f robot_state_publisher
pkill -9 -x static_transform_publisher

echo "[5/8] 停止 RViz 与 Teleop..."
pkill -9 -f rviz2
pkill -9 -f teleop_twist

echo "[6/8] 停止其它 ROS2 Launch/Run..."
pkill -9 -f "ros2 launch"
pkill -9 -f "ros2 run"

echo "[7/8] 等待进程退出..."
sleep 2

echo "[8/8] 开始验证..."

# -----------------------------
# 环境
# -----------------------------
source /opt/ros/humble/setup.bash
source /home/dex/dexe_mobile_nav/install/setup.bash

export ROS_LOCALHOST_ONLY=1

# -----------------------------
# 检查残余进程
# -----------------------------
LEFT_PROCESS=$(pgrep -af "gzserver|sim.launch|bringup_launch|component_container|wheel_odometry|wheel_cmd|robot_state_publisher|static_transform" | grep -v grep)

# -----------------------------
# 检查ROS节点
# -----------------------------
LEFT_NODE=$(ros2 node list)

echo ""
echo "================ 验证结果 ================"

if [[ -z "$LEFT_PROCESS" && -z "$LEFT_NODE" ]]; then
    echo ""
    echo "=========================================="
    echo "✅ 全部清理完成！"
    echo "✓ 无残余进程"
    echo "✓ ROS2 Node为空"
    echo "=========================================="
else
    echo ""
    echo "⚠ 检测到残余内容！"

    if [[ -n "$LEFT_PROCESS" ]]; then
        echo ""
        echo "【残余进程】"
        echo "$LEFT_PROCESS"
    fi

    if [[ -n "$LEFT_NODE" ]]; then
        echo ""
        echo "【残余ROS节点】"
        echo "$LEFT_NODE"
    fi

    echo ""
    echo "建议重新执行一次脚本。"
    echo "如果仍存在残余，请检查是否有新的节点名称未加入 pkill 列表。"
    echo "=========================================="
fi
