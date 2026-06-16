#!/bin/bash

# Mission Manager 快速集成测试脚本
# 自动启动所有相关节点并运行集成测试

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="/home/dex/workspace_pzc"

echo "========================================="
echo "Mission Manager 快速集成测试"
echo "========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查是否source了workspace
if [ -z "$AMENT_PREFIX_PATH" ]; then
    echo -e "${YELLOW}⚠️  检测到未source workspace，正在source...${NC}"
    cd "$WORKSPACE_DIR"
    source install/setup.bash
fi

# 清理函数
cleanup() {
    echo ""
    echo -e "${YELLOW}正在清理...${NC}"
    
    # 停止所有后台进程
    if [ ! -z "$MAP_MANAGER_PID" ]; then
        echo "停止 map_manager (PID: $MAP_MANAGER_PID)"
        kill $MAP_MANAGER_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$NAV_MANAGER_PID" ]; then
        echo "停止 nav_point_manager (PID: $NAV_MANAGER_PID)"
        kill $NAV_MANAGER_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$NAV_ADAPTER_PID" ]; then
        echo "停止 nav_adapter (PID: $NAV_ADAPTER_PID)"
        kill $NAV_ADAPTER_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$MISSION_MANAGER_PID" ]; then
        echo "停止 mission_manager (PID: $MISSION_MANAGER_PID)"
        kill $MISSION_MANAGER_PID 2>/dev/null || true
    fi
    
    # 等待进程结束
    sleep 2
    
    echo -e "${GREEN}✅ 清理完成${NC}"
}

# 捕获退出信号
trap cleanup EXIT INT TERM

# 检查包是否已编译
echo "📦 检查包编译状态..."
PACKAGES=("map_manager_ros2" "nav2_point_manager" "mission_manager" "nav_adapter")
for pkg in "${PACKAGES[@]}"; do
    if [ ! -d "$WORKSPACE_DIR/install/$pkg" ]; then
        echo -e "${RED}❌ 包 $pkg 未编译${NC}"
        echo "请先编译: colcon build --packages-select $pkg"
        exit 1
    fi
    echo -e "${GREEN}✅ $pkg${NC}"
done

echo ""
echo "========================================="
echo "第1步: 启动所有节点"
echo "========================================="
echo ""

# 设置测试地图文件夹路径
TEST_MAPS_DIR="/home/dex/workspace_pzc/dexe_mobile_nav/test_maps"

# 1. 启动 map_manager (with map_folder parameter)
echo "🚀 启动 map_manager_ros2 (map_folder: $TEST_MAPS_DIR)..."
ros2 run map_manager_ros2 map_manager_node --ros-args -p map_folder:=$TEST_MAPS_DIR > /tmp/map_manager.log 2>&1 &
MAP_MANAGER_PID=$!
echo -e "${GREEN}✅ map_manager_ros2 启动 (PID: $MAP_MANAGER_PID)${NC}"
sleep 2

# 2. 启动 nav_point_manager (with map_folder and map_name parameters)
echo "🚀 启动 nav2_point_manager (map_folder: $TEST_MAPS_DIR)..."
ros2 run nav2_point_manager nav2_point_manager_node --ros-args -p map_folder:=$TEST_MAPS_DIR -p map_name:=test_map > /tmp/nav_point_manager.log 2>&1 &
NAV_MANAGER_PID=$!
echo -e "${GREEN}✅ nav2_point_manager 启动 (PID: $NAV_MANAGER_PID)${NC}"
sleep 2

# 3. 启动 nav_adapter
echo "🚀 启动 nav_adapter..."
ros2 run nav_adapter nav_adapter_node > /tmp/nav_adapter.log 2>&1 &
NAV_ADAPTER_PID=$!
echo -e "${GREEN}✅ nav_adapter 启动 (PID: $NAV_ADAPTER_PID)${NC}"
sleep 2

# 4. 启动 mission_manager (单独启动，避免 executor 冲突)
echo "🚀 启动 mission_manager..."
echo -e "${YELLOW}注意: 使用单独启动方式避免 executor 冲突${NC}"
ros2 run mission_manager mission_manager_node > /tmp/mission_manager.log 2>&1 &
MISSION_MANAGER_PID=$!
echo -e "${GREEN}✅ mission_manager 启动 (PID: $MISSION_MANAGER_PID)${NC}"
echo -e "${GREEN}   使用 MultiThreadedExecutor (4 线程)${NC}"
sleep 3

echo ""
echo "========================================="
echo "第2步: 检查节点状态"
echo "========================================="
echo ""

# 检查节点是否运行
echo "📊 检查ROS2节点..."
NODES=$(ros2 node list 2>/dev/null)

if echo "$NODES" | grep -q "map_manager"; then
    echo -e "${GREEN}✅ map_manager 节点运行中${NC}"
else
    echo -e "${RED}❌ map_manager 节点未找到${NC}"
    echo "查看日志: cat /tmp/map_manager.log"
fi

if echo "$NODES" | grep -q "nav_point_manager"; then
    echo -e "${GREEN}✅ nav_point_manager 节点运行中${NC}"
else
    echo -e "${RED}❌ nav_point_manager 节点未找到${NC}"
    echo "查看日志: cat /tmp/nav_point_manager.log"
fi

if echo "$NODES" | grep -q "nav_adapter"; then
    echo -e "${GREEN}✅ nav_adapter 节点运行中${NC}"
else
    echo -e "${RED}❌ nav_adapter 节点未找到${NC}"
    echo "查看日志: cat /tmp/nav_adapter.log"
fi

if echo "$NODES" | grep -q "mission_manager"; then
    echo -e "${GREEN}✅ mission_manager 节点运行中${NC}"
else
    echo -e "${RED}❌ mission_manager 节点未找到${NC}"
    echo "查看日志: cat /tmp/mission_manager.log"
fi

echo ""
echo "========================================="
echo "第3步: 运行集成测试"
echo "========================================="
echo ""

# 等待所有服务就绪
echo "⏳ 等待服务就绪..."
sleep 3

# 运行集成测试
echo "🧪 运行集成测试脚本..."
echo ""

python3 "$SCRIPT_DIR/test_integration.py"
TEST_RESULT=$?

echo ""
echo "========================================="
echo "测试结果"
echo "========================================="
echo ""

if [ $TEST_RESULT -eq 0 ]; then
    echo -e "${GREEN}🎉 集成测试通过！${NC}"
else
    echo -e "${RED}❌ 集成测试失败${NC}"
    echo ""
    echo "查看日志:"
    echo "  map_manager: cat /tmp/map_manager.log"
    echo "  nav_point_manager: cat /tmp/nav_point_manager.log"
    echo "  mission_manager: cat /tmp/mission_manager.log"
fi

echo ""
echo "========================================="
echo "可选: 手动测试命令"
echo "========================================="
echo ""
echo "如果您想手动测试，节点仍在运行中。"
echo "按 Ctrl+C 停止所有节点。"
echo ""
echo "手动测试示例:"
echo "  # 获取地图列表"
echo "  ros2 service call /map_manager/get_map_list nav_interfaces/srv/GetMapList"
echo ""
echo "  # 获取所有导航点"
echo "  ros2 service call /nav_point_manager/get_all_nav_points nav_interfaces/srv/GetAllNavPoints"
echo ""
echo "  # 获取任务管理器状态"
echo "  ros2 service call /mission_manager/get_mission_status nav_interfaces/srv/GetStatus"
echo ""

# 等待用户中断或测试失败时自动退出
if [ $TEST_RESULT -eq 0 ]; then
    echo -e "${YELLOW}按 Ctrl+C 停止所有节点...${NC}"
    wait
else
    echo -e "${YELLOW}测试失败，5秒后自动退出...${NC}"
    sleep 5
    exit 1
fi

