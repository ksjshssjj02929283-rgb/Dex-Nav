#!/usr/bin/env bash
set -e  # 出错立即退出

WORKSPACE_DIR=$(pwd)
INSTALL_DIR="${WORKSPACE_DIR}/install"
OUTPUT_DIR="${WORKSPACE_DIR}/release"
TAR_PATH="${OUTPUT_DIR}/install.tar.gz"

# === 添加：记录编译开始时间 ===
BUILD_START=$(date +%s)

source /opt/ros/humble/setup.bash

# === 1. 检查 ROS 环境 ===
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ 请先 source 对应ROS环境，例如："
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi
echo "✅ ROS版本: $ROS_DISTRO"

# === 2. 编译 ===
echo "⚙️ 开始 colcon build ..."

colcon build --packages-up-to nav2_graceful_controller --parallel-workers 8
colcon build --packages-up-to nav2_bringup --parallel-workers 8
colcon build --packages-up-to dexe_sensor_process
source ${INSTALL_DIR}/setup.bash
colcon build

# === 3. 打包 install 目录 ===
mkdir -p "$OUTPUT_DIR"
echo "🧹 清理旧的压缩包..."
rm -f "$TAR_PATH"

echo "📦 正在打包 install 目录为 install.tar.gz..."
tar -czf "$TAR_PATH" -C "$WORKSPACE_DIR" install

# === 添加：计算编译时间 ===
BUILD_END=$(date +%s)
BUILD_DURATION=$((BUILD_END - BUILD_START))

# 格式化输出
MIN=$((BUILD_DURATION / 60))
SEC=$((BUILD_DURATION % 60))

# === 4. 完成 ===
echo "==============================================="
echo "✅ 编译完成并打包成功！"
echo "📁 输出路径：$TAR_PATH"
echo "⏱️ 总耗时：${MIN} 分 ${SEC} 秒"
echo "==============================================="
