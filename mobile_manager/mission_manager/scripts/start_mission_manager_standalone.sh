#!/bin/bash

# 单独启动 mission_manager 节点
# 这样可以避免 launch 系统的执行器冲突

set -e

echo "=========================================="
echo "  单独启动 Mission Manager"
echo "=========================================="

# 检查工作空间
if [ ! -f install/setup.bash ]; then
    echo "错误: 找不到 install/setup.bash"
    echo "请先编译: colcon build --packages-select mission_manager"
    exit 1
fi

# Source 环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "✅ 环境已配置"
echo ""
echo "启动 mission_manager_node..."
echo "注意: 此节点将使用 MultiThreadedExecutor (4 线程)"
echo ""
echo "提示: 此节点需要以下节点已在运行:"
echo "  - nav_adapter_node (用于导航)"
echo "  - nav_point_manager_node (用于获取导航点)"
echo "  - map_manager (可选，用于地图管理)"
echo ""
echo "在另一个终端运行:"
echo "  ros2 launch mission_manager managers_bringup.launch.py"
echo ""
echo "=========================================="

# 启动节点
ros2 run mission_manager mission_manager_node

