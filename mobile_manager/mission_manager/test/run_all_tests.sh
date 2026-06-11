#!/bin/bash
# Mission Manager 测试脚本
# 运行所有测试用例

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 获取脚本目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Mission Manager 测试套件${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 测试1: 参数文件测试
echo -e "${YELLOW}[1/2] 测试参数文件...${NC}"
if python3 "$SCRIPT_DIR/test_params.py"; then
    echo -e "${GREEN}✅ 参数文件测试通过${NC}"
else
    echo -e "${RED}❌ 参数文件测试失败${NC}"
    exit 1
fi

echo ""

# 测试2: ROS2节点测试
echo -e "${YELLOW}[2/2] 测试ROS2节点接口...${NC}"
echo -e "${YELLOW}注意: 此测试需要mission_manager节点运行${NC}"
echo -e "${YELLOW}请在另一个终端运行: ros2 launch mission_manager mission_manager.launch.py${NC}"
echo ""
read -p "mission_manager节点是否已启动? (y/N) " -n 1 -r
echo

if [[ $REPLY =~ ^[Yy]$ ]]; then
    if python3 "$SCRIPT_DIR/test_mission_manager.py"; then
        echo -e "${GREEN}✅ ROS2节点测试通过${NC}"
    else
        echo -e "${RED}❌ ROS2节点测试失败${NC}"
        exit 1
    fi
else
    echo -e "${YELLOW}⏭  跳过ROS2节点测试${NC}"
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}所有测试完成!${NC}"
echo -e "${GREEN}========================================${NC}"

