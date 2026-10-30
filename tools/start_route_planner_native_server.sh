#!/usr/bin/env bash
# Copyright 2026 DEXE Robot Engineer
# SPDX-License-Identifier: Apache-2.0

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "==> 加载 ROS 2 Humble 环境..."
source /opt/ros/humble/setup.bash

echo "==> 加载工作空间环境..."
if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
else
  echo "警告: ${WORKSPACE_DIR}/install/setup.bash 不存在，请先执行 colcon build"
fi

echo "==> 启动本地原生 C++ 规划桥接服务..."
exec /usr/bin/python3 "${SCRIPT_DIR}/route_planner_native_server.py"
