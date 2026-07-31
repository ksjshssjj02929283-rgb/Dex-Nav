#!/usr/bin/env bash
set -e  # 出错立即退出

WORKSPACE_DIR=$(pwd)
INSTALL_DIR="${WORKSPACE_DIR}/install"
OUTPUT_DIR="${WORKSPACE_DIR}/release"
TAR_PATH="${OUTPUT_DIR}/install.tar.gz"

# === 添加：记录编译开始时间 ===
BUILD_START=$(date +%s)

source /opt/ros/humble/setup.bash

resolve_build_simulation() {
    local mode="${1,,}"
    local architecture="$2"

    case "$mode" in
        auto)
            case "$architecture" in
                aarch64|arm64|armv7l|armv8l)
                    echo "false"
                    ;;
                *)
                    echo "true"
                    ;;
            esac
            ;;
        1|true|on|yes)
            echo "true"
            ;;
        0|false|off|no)
            echo "false"
            ;;
        *)
            echo "❌ BUILD_SIMULATION 仅支持 auto、1/true/on/yes 或 0/false/off/no，当前值: $1" >&2
            return 2
            ;;
    esac
}

# === 1. 检查 ROS 环境 ===
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ 请先 source 对应ROS环境，例如："
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi
echo "✅ ROS版本: $ROS_DISTRO"

# === 2. 检查并同步子模块 ===
if [ -f "${WORKSPACE_DIR}/.gitmodules" ]; then
    echo "🔄 检测到 .gitmodules，正在同步子模块..."
    git submodule sync --recursive
    git submodule update --init --recursive
    echo "✅ 子模块已更新完成。"
else
    echo "ℹ️ 未检测到 .gitmodules，跳过子模块更新。"
fi

# === 3. 确定是否编译 Gazebo 仿真包 ===
ARCHITECTURE=$(uname -m)
BUILD_SIMULATION_MODE="${BUILD_SIMULATION:-auto}"
if ! BUILD_SIMULATION_ENABLED=$(resolve_build_simulation "$BUILD_SIMULATION_MODE" "$ARCHITECTURE"); then
    exit 2
fi

COLCON_BUILD_ARGS=(--parallel-workers 8)
SIMULATION_PACKAGES=()

if [ "$BUILD_SIMULATION_ENABLED" = "false" ]; then
    SIMULATION_DIR="${WORKSPACE_DIR}/mobile_simulation"
    if [ -d "$SIMULATION_DIR" ]; then
        SIMULATION_PACKAGE_OUTPUT=$(colcon list --base-paths "$SIMULATION_DIR" --names-only)
        while IFS= read -r package_name; do
            if [ -n "$package_name" ]; then
                # 0.4.6 transition: w1_nav2 still installs the physical-robot
                # launch entry. In 0.5.0 that entry moves to nav2_bringup and
                # the remaining simulation package will be renamed w1_sim.
                if [ "$package_name" = "w1_nav2" ]; then
                    continue
                fi
                SIMULATION_PACKAGES+=("$package_name")
            fi
        done <<< "$SIMULATION_PACKAGE_OUTPUT"
    fi

    if [ ${#SIMULATION_PACKAGES[@]} -gt 0 ]; then
        COLCON_BUILD_ARGS+=(--packages-skip "${SIMULATION_PACKAGES[@]}")
    fi
fi

echo "ℹ️ 架构: $ARCHITECTURE；BUILD_SIMULATION=$BUILD_SIMULATION_MODE；编译仿真包: $BUILD_SIMULATION_ENABLED"
if [ ${#SIMULATION_PACKAGES[@]} -gt 0 ]; then
    echo "⏭️ 跳过 mobile_simulation 包: ${SIMULATION_PACKAGES[*]}"
fi

# === 4. 确保 nav_interfaces 可用 ===
if ros2 pkg list 2>/dev/null | grep -q "nav_interfaces"; then
    echo "✅ 找到 nav_interfaces 包，跳过 dexe_interfaces 编译。"
else
    echo "⚠️ 未找到 nav_interfaces，正在编译 dexe_interfaces..."
    colcon build --base-paths dexe_interfaces "${COLCON_BUILD_ARGS[@]}"
    source "${INSTALL_DIR}/setup.bash"
    echo "✅ dexe_interfaces 编译完成。"
fi

# === 5. 编译 ===
echo "⚙️ 开始 colcon build ..."

colcon build --packages-up-to nav2_graceful_controller "${COLCON_BUILD_ARGS[@]}"
colcon build --packages-up-to nav2_bringup "${COLCON_BUILD_ARGS[@]}"
colcon build --packages-up-to dexe_sensor_process "${COLCON_BUILD_ARGS[@]}"
source "${INSTALL_DIR}/setup.bash"
if [ "$BUILD_SIMULATION_ENABLED" = "false" ]; then
    echo "⚙️ 显式编译 0.4.6 真机启动过渡包 w1_nav2 ..."
    colcon build --packages-select w1_nav2 "${COLCON_BUILD_ARGS[@]}"
fi
colcon build "${COLCON_BUILD_ARGS[@]}"

# === 6. 打包 install 目录 ===
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

# === 7. 完成 ===
echo "==============================================="
echo "✅ 编译完成并打包成功！"
echo "📁 输出路径：$TAR_PATH"
echo "⏱️ 总耗时：${MIN} 分 ${SEC} 秒"
echo "==============================================="
