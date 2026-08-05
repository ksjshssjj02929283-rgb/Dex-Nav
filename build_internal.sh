#!/usr/bin/env bash
set -e  # 出错立即退出

WORKSPACE_DIR="/workspace"
OUTPUT_DIR="${WORKSPACE_DIR}/release"
TAR_PATH="${OUTPUT_DIR}/install_arm64.tar.gz"

echo "=== 1. 初始化 ROS 2 容器环境 ==="
source /opt/ros/humble/setup.bash
echo "✅ ROS版本: $ROS_DISTRO (aarch64)"

# === 2. 同步 rosdepc 并自动安装缺少的第三方 ARM64 依赖 ===
# echo "=== 2. 安装缺失的 ARM64 依赖 (rosdepc) ==="
# rosdepc update
# 使用 -r 即使有包没有定义 rosdep key 也能继续安装其他依赖
# rosdepc install --from-paths . --ignore-src -y -r --rosdistro humble11

# === 3. 配置 colcon 编译选项 ===
export MAKEFLAGS="-j6"
export CMAKE_BUILD_PARALLEL_LEVEL="6"
COLCON_ARGS="--event-handlers console_direct+ --cmake-args -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=87"
# 隔离的 arm64 编译路径，避免与 x86 混淆
BUILD_BASE="build_arm64"
INSTALL_BASE="install_arm64"
LOG_BASE="log_arm64"

# === 4. 开始编译 ===
echo "=== 3. 正在编译 dexe_interfaces (ARM64) ==="
# colcon --log-base "$LOG_BASE" build --build-base "$BUILD_BASE" --install-base "$INSTALL_BASE" --base-paths dexe_interfaces --parallel-workers 2 ${COLCON_ARGS}
source ${WORKSPACE_DIR}/${INSTALL_BASE}/setup.bash

echo "=== 4. 分步编译核心包 (ARM64) ==="
# colcon --log-base "$LOG_BASE" build --build-base "$BUILD_BASE" --install-base "$INSTALL_BASE" --packages-up-to nav2_graceful_controller --parallel-workers 2 ${COLCON_ARGS}
# colcon --log-base "$LOG_BASE" build --build-base "$BUILD_BASE" --install-base "$INSTALL_BASE" --packages-up-to nav2_bringup --parallel-workers 6 ${COLCON_ARGS}
# colcon --log-base "$LOG_BASE" build --build-base "$BUILD_BASE" --install-base "$INSTALL_BASE" --packages-up-to dexe_sensor_process --parallel-workers 2 ${COLCON_ARGS}

source ${WORKSPACE_DIR}/${INSTALL_BASE}/setup.bash
# echo "=== 5. 编译其余全部包 (ARM64) ==="
# colcon --log-base "$LOG_BASE" build --build-base "$BUILD_BASE" --install-base "$INSTALL_BASE" ${COLCON_ARGS}
echo "=== 5. 编译其余全部包 (ARM64) ==="
# 适当降低并行包数量到 2（Ninja 会在单个包内自动使用所有多核），避免线程再度过载
colcon --log-base "$LOG_BASE" build --build-base "$BUILD_BASE" --install-base "$INSTALL_BASE" --packages-up-to nav2_mppi_controller --parallel-workers 2 ${COLCON_ARGS}


# === 5. 导出编译成果 ===
mkdir -p "$OUTPUT_DIR"
echo "=== 6. 打包 install_arm64 目录 ==="
rm -f "$TAR_PATH"
tar -czf "$TAR_PATH" -C "$WORKSPACE_DIR" "$INSTALL_BASE"
echo "✅ 已生成 ARM64 压缩包: $TAR_PATH"

# （注：Debian 部署包的打包功能已应要求移除，仅保留 tar.gz 压缩包输出）

echo "==============================================="
echo "✅ ARM64 交叉编译和打包已顺利完成！"
echo "📁 压缩包路径：$TAR_PATH"
echo "==============================================="
