#!/usr/bin/env bash
# run_sim.sh - 一键清理并启动仿真与导航的整合脚本

# 确保在出错时不会直接导致终端崩溃退出
set -e

echo "======================================================"
echo "🧹 1. 正在强力清理残留的 ROS 2、Gazebo 及 RViz 进程..."
echo "======================================================"

# 强杀所有相关的物理仿真与中间件容器进程
pkill -9 -f ros2 || true
pkill -9 -f gazebo || true
pkill -9 -f gzserver || true
pkill -9 -f gzclient || true
pkill -9 -f rviz2 || true
pkill -9 -f rviz || true
pkill -9 -f component_container || true
pkill -9 -f component_container_isolated || true
pkill -9 -f robot_state_publisher || true
pkill -9 -f static_transform_publisher || true
pkill -9 -f map_server || true
pkill -9 -f amcl || true
pkill -9 -f fake_amcl.py || true
pkill -9 -f lifecycle_manager || true
pkill -9 -f planner_server || true
pkill -9 -f controller_server || true
pkill -9 -f smoother_server || true
pkill -9 -f behavior_server || true
pkill -9 -f bt_navigator || true
pkill -9 -f waypoint_follower || true
pkill -9 -f velocity_smoother || true

echo "🧼 2. 正在清空 Fast-DDS 共享内存锁、Gazebo 缓存及 log.log..."
rm -rf /dev/shm/fastrtps_* || true
rm -rf ~/.gazebo/log/* /tmp/gazebo* || true
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
> "${WORKSPACE_DIR}/log.log" || true
echo "   缓存与日志清理完毕。"

echo "🔄 3. 正在加载 ROS 2 运行环境与仿真环境变量..."
# 过滤掉 PATH 中所有带 conda/miniconda 的路径，防止 Python 3.14 干扰，但保留显卡等其他系统路径
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -E "conda|miniconda" | paste -sd: -s)
export PATH=/opt/ros/humble/bin:/usr/bin:/bin:/usr/sbin:/sbin:${PATH}

unset PYTHONPATH
unset PYTHONHOME
unset CONDA_PREFIX
unset CONDA_DEFAULT_ENV
unset CONDA_PYTHON_EXE
unset CONDA_SHLVL

if type conda &>/dev/null; then
    conda deactivate 2>/dev/null || true
fi

export PYTHON_EXECUTABLE=/usr/bin/python3

# 自动检测并补全 DISPLAY 环境变量，授权 X11 屏幕连接
if [ -z "$DISPLAY" ]; then
    # 自动探测系统当前活跃的 X11 Display（例如 :1, :0 等）
    ACTIVE_DISPLAY=$(ls /tmp/.X11-unix/ 2>/dev/null | grep -oE '[0-9]+' | head -n 1)
    if [ -n "$ACTIVE_DISPLAY" ]; then
        export DISPLAY=":${ACTIVE_DISPLAY}"
    else
        export DISPLAY=:0
    fi
fi
xhost +local: 2>/dev/null || true

# 设置 Gazebo 离线运行防超时卡死，以及 QT 平台兼容与 OGRE 渲染兼容
export GAZEBO_MODEL_DATABASE_URI=""
export QT_QPA_PLATFORM=xcb
export OGRE_RTT_MODE=Copy

# 获取当前工作空间绝对路径
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 优先 source 系统 ROS 2 环境
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# 显式载入系统 Gazebo 环境变量，防止 Ogre 渲染和插件库寻址冲突导致渲染相机崩溃
if [ -f "/usr/share/gazebo/setup.sh" ]; then
    source /usr/share/gazebo/setup.sh
elif [ -f "/usr/share/gazebo-11/setup.sh" ]; then
    source /usr/share/gazebo-11/setup.sh
fi

# 载入本地编译的工作空间环境
if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
    echo "✅ 环境加载成功：已 source ${WORKSPACE_DIR}/install/setup.bash"
else
    echo "❌ 错误：未在当前目录下找到 install/setup.bash，请确保已执行过 ./build.sh 编译！"
    exit 1
fi

echo "🚀 4. 正在拉起仿真与导航..."
# 支持将脚本传入的其他参数透传给 launch 文件（例如 ./run_sim.sh headless:=True）
ros2 launch tb3_sim tb3_simulation_launch.py use_simulator:=True use_sim_time:=True use_composition:=False "$@"


