#!/usr/bin/env bash
set -e  # 出错立即退出

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
SOURCE_ROOT="${SCRIPT_DIR}"
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

run_colcon_build() {
    printf '▶️ 执行编译命令：'
    printf ' %q' colcon build "$@"
    printf '\n'
    colcon build "$@"
}

# === 1. 检查 ROS 环境 ===
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ 请先 source 对应ROS环境，例如："
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi
echo "✅ ROS版本: $ROS_DISTRO"
echo "ℹ️ 构建工作区: $WORKSPACE_DIR；源码根目录: $SOURCE_ROOT"

# === 2. 检查并同步子模块 ===
if [ -f "${SOURCE_ROOT}/.gitmodules" ]; then
    echo "🔄 检测到 .gitmodules，正在同步子模块..."
    git -C "$SOURCE_ROOT" submodule sync --recursive
    git -C "$SOURCE_ROOT" submodule update --init --recursive
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

COLCON_COMMON_BUILD_ARGS=(
    --parallel-workers 8
    --cmake-args
    -DCMAKE_BUILD_TYPE=Release
)
COLCON_BUILD_ARGS=("${COLCON_COMMON_BUILD_ARGS[@]}")
SIMULATION_PACKAGES=()

if [ "$BUILD_SIMULATION_ENABLED" = "false" ]; then
    # build.sh 既允许在源码根内执行，也允许从外层 colcon 工作区以
    # ./dexe_mobile_nav/build.sh 调用；仿真包发现必须锚定脚本所在源码树。
    SIMULATION_DIR="${SOURCE_ROOT}/mobile_simulation"
    if [ -d "$SIMULATION_DIR" ]; then
        SIMULATION_PACKAGE_OUTPUT=$(colcon list --base-paths "$SIMULATION_DIR" --names-only)
        while IFS= read -r package_name; do
            # 0.4.6 过渡期：w1_nav2 仍承载真机启动入口
            # w1_nav2_actual.launch.py，因此 ARM 构建不能把它作为仿真包跳过。
            # 0.5.0 再将真机入口迁入 nav2_bringup，并把仿真包改名为 w1_sim。
            if [ -n "$package_name" ] && [ "$package_name" != "w1_nav2" ]; then
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
    run_colcon_build --base-paths dexe_interfaces "${COLCON_BUILD_ARGS[@]}"
    source "${INSTALL_DIR}/setup.bash"
    echo "✅ dexe_interfaces 编译完成。"
fi

# === 5. 编译 ===
echo "⚙️ 开始 colcon build ..."

run_colcon_build --packages-up-to nav2_graceful_controller "${COLCON_BUILD_ARGS[@]}"
run_colcon_build --packages-up-to nav2_bringup "${COLCON_BUILD_ARGS[@]}"
run_colcon_build --packages-up-to dexe_sensor_process "${COLCON_BUILD_ARGS[@]}"
source "${INSTALL_DIR}/setup.bash"

if [ "$BUILD_SIMULATION_ENABLED" = "false" ]; then
    echo "⚙️ 0.4.6 ARM 过渡：显式编译包含真机入口的 w1_nav2 ..."
    run_colcon_build --packages-select w1_nav2 "${COLCON_BUILD_ARGS[@]}"
    source "${INSTALL_DIR}/setup.bash"
fi

# 最终全工作区增量构建不重复选择前面已经显式构建的 w1_nav2。
FINAL_SKIP_PACKAGES=("${SIMULATION_PACKAGES[@]}")
if [ "$BUILD_SIMULATION_ENABLED" = "false" ]; then
    FINAL_SKIP_PACKAGES+=("w1_nav2")
fi

FINAL_COLCON_BUILD_ARGS=("${COLCON_COMMON_BUILD_ARGS[@]}")
if [ ${#FINAL_SKIP_PACKAGES[@]} -gt 0 ]; then
    FINAL_COLCON_BUILD_ARGS+=(--packages-skip "${FINAL_SKIP_PACKAGES[@]}")
fi
run_colcon_build "${FINAL_COLCON_BUILD_ARGS[@]}"

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
