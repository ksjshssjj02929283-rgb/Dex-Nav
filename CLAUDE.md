# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 构建命令

不允许直接构建，所有构建都需要询问用户

### 完整构建（必须按顺序执行四步）

```bash
# build.sh 内部的顺序不可颠倒，原因是包间存在依赖关系
colcon build --packages-up-to nav2_graceful_controller --parallel-workers 8
colcon build --packages-up-to nav2_bringup --parallel-workers 8
colcon build --packages-up-to dexe_sensor_process
source install/setup.bash
colcon build
```

也可直接运行 `./build.sh`（自动处理顺序并打包为 `release/install.tar.gz`）。

### 单包构建

```bash
colcon build --packages-select mobile_perception --parallel-workers 8
```

### 测试

```bash
colcon test --packages-select <package_name> --event-handlers console_direct+
colcon test-result --verbose
# 或直接运行编译后的测试可执行文件
./install/<pkg>/lib/<pkg>/test_<name>
```

### 代码风格检查

```bash
python3 utils/check_style.py          # 检查
python3 utils/check_style.py --apply  # 自动修复
```

## 系统架构

### 包职责划分

| 包 | 职责 |
|---|---|
| `mobile_perception` | 底层运动控制、AprilTag检测、充电对接 |
| `mobile_navigation` | Nav2 导航堆栈（含多处定制修改） |
| `dexe_sensor_process` | RGBD → 点云 → 激光扫描 的感知管道 |

### 导航数据流

```
RGBD相机 → dexe_sensor_process (点云滤波/地面检测) → 激光扫描
                                                          ↓
                                          nav2_amcl (定位) + nav2_costmap_2d
                                                          ↓
                                    nav2_smac_planner (全局路径规划)
                                                          ↓
                              nav2_graceful_controller / nav2_regulated_pure_pursuit
                                                          ↓
                                          mobile_control_node (Lyapunov底层控制)
                                                          ↓
                                                    /control/cmd_vel
```

### 充电对接流程

AprilTag标签检测（`apriltag_detection_node`）→ 精确位姿估计 → `charging_pose_service_node` 保存充电点 → `undocking.py` 离桩控制。`goal_controller.py` 实现 Lyapunov 控制律，C++ 节点 `mobile_control_node` 是其 ROS 封装。

### 关键配置文件

- **活跃导航参数**：`mobile_navigation/nav2_bringup/params/w1_nav2_params.yaml`（W1机器人专用，非默认的 `nav2_params.yaml`）
- **移动控制参数**：`mobile_perception/config/mobile_control_params.yaml`（Lyapunov 增益 k_rho/k_alpha/k_beta 等）
- **相机配置**：`mobile_perception/config/camera_config.json`（多分辨率版本按环境切换）
- **对接配置**：`mobile_perception/config/docking_config.json`

### 对 Nav2 的定制修改

`mobile_navigation/` 下的 Nav2 包并非原版，已针对项目做出以下修改：
- `nav2_smac_planner`：尾段路径优化（AGX 导航精度调优）
- `nav2_controller`：增加控制接口日志打印
- `nav2_graceful_controller`：项目自定义实现（非上游版本）
- `nav2_bt_navigator`：修复 `navigate_through_poses` 中经过点与全局规划同频问题
- `nav2_mppi_controller`：基于 xtensor（非上游 Eigen），已移植加速度限制，详见下方说明

修改 Nav2 内部包时需注意：上游变更不会自动合并，需手动 cherry-pick。

### nav2_mppi_controller 定制说明

> 完整与上游 main 分支的差异见 `mobile_navigation/nav2_mppi_controller/DIFF_FROM_MAIN.md`

**底层库**：使用 xtensor（上游已迁移至 Eigen，本项目不跟进，架构改动过大）

**已移植的 main 分支功能**：

- ✅ **加速度限制**（ax_max/ax_min/ay_max/ay_min/az_max）：在 `predict()` 逐步裁剪每个时间步的速度变化量，使候选轨迹物理可行

**尚未移植**：SG滤波调用位置、open_loop 模式、VelocityDeadbandCritic、kinematic_guard

**MPPI 加速度参数配置**（`w1_nav2_params.yaml` 中 FollowPath 节）：

```yaml
ax_max: 0.5      # 前进最大加速度 m/s²
ax_min: -0.5     # 最大减速度（必须为负数）
ay_max: 0.5      # 侧向加速度（差速机器人不生效）
ay_min: -0.5
az_max: 1.0      # 最大角加速度 rad/s²
```

> `ax_max: 0.0` 可禁用加速度约束，退化为原始行为。

### 版权头

所有新建源文件必须包含：

```cpp
// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------
```

Python 文件同理，使用 `#` 注释。
