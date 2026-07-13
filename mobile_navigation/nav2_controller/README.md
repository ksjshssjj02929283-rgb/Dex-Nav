# nav2_controller 说明

```text
书写日期：2026-06-15
书写人：zhangzhihao
```

`nav2_controller` 是 Nav2 导航链路中的控制层，主要负责执行 `nav2_msgs::action::FollowPath` 动作。

通俗理解：

```text
planner_server 负责算出路径
controller_server 负责沿着路径生成速度指令
goal_checker 负责判断是否到达目标
progress_checker 负责判断机器人是否还在有效前进
```

本仓库中的 `nav2_controller` 在上游 Nav2 基础上增加了若干本地改动，重点包括：

```text
1. 近目标进展检查 near_goal_progress
2. 支持纵向/横向误差拆分的 goal_checker
3. 支持 1cm 优先、2cm 退让接受的高精度 goal_checker
```

## 1. 代码结构

```text
nav2_controller/
├── src/
│   ├── main.cpp
│   └── controller_server.cpp
│
├── plugins/
│   ├── simple_progress_checker.cpp
│   ├── pose_progress_checker.cpp
│   ├── simple_goal_checker.cpp
│   ├── stopped_goal_checker.cpp
│   ├── longitudinal_goal_checker.cpp
│   └── precision_longitudinal_goal_checker.cpp
│
├── include/nav2_controller/
│   ├── controller_server.hpp
│   └── plugins/
│       ├── simple_progress_checker.hpp
│       ├── pose_progress_checker.hpp
│       ├── simple_goal_checker.hpp
│       ├── stopped_goal_checker.hpp
│       ├── longitudinal_goal_checker.hpp
│       └── precision_longitudinal_goal_checker.hpp
│
├── plugins.xml
├── CMakeLists.txt
└── plugins/test/
```

目录分工：

```text
src/
  controller_server 主流程，不建议把插件实现放到这里。

plugins/
  controller、progress_checker、goal_checker 等插件实现。

include/nav2_controller/plugins/
  插件头文件。

plugins.xml
  pluginlib 插件注册文件。
```

## 2. Controller Server 执行顺序

`controller_server` 收到 `FollowPath` action 后，大致执行顺序如下：

```text
FollowPath action
└── 选择 controller 插件
└── 选择 goal_checker 插件
└── setPlannerPath()
    ├── 把 path 传给 controller
    ├── 记录路径终点 end_pose_
    └── reset 当前 goal_checker
└── 控制循环 computeControl()
    ├── updateGlobalPath()
    ├── computeAndPublishVelocity()
    │   ├── 检查 progress_checker
    │   ├── 近目标时应用 near_goal_progress
    │   ├── 调用 controller 计算速度
    │   └── 发布 cmd_vel
    └── isGoalReached()
        └── 调用当前 goal_checker 判断是否到达目标
```

注意：

```text
goal_checker 不生成速度，它只返回 true 或 false。
如果 goal_checker 返回 false，controller_server 会继续控制。
如果 goal_checker 返回 true，controller_server 会发布零速度并结束当前 FollowPath。
```

## 3. 已注册插件

插件注册在 `plugins.xml` 中。

### 3.1 Progress Checker

```text
nav2_controller::SimpleProgressChecker
```

只判断机器人是否移动了足够距离。

```text
nav2_controller::PoseProgressChecker
```

在距离判断基础上增加角度判断。当前 `w1_nav2_params.yaml` 使用的是这个插件。

常见参数：

```yaml
progress_checker:
  plugin: "nav2_controller::PoseProgressChecker"
  required_movement_radius: 0.1
  required_movement_angle: 0.01
  movement_time_allowance: 3.0
```

含义：

```text
required_movement_radius:
  在允许时间内至少移动多少距离才算有进展。

required_movement_angle:
  在允许时间内至少转动多少角度也可以算有进展。

movement_time_allowance:
  多久没有足够进展后认为控制失败。
```

### 3.2 Goal Checker

```text
nav2_controller::SimpleGoalChecker
```

默认简单到点判断，主要看当前位置和目标点的 XY 距离，以及 yaw 误差。

```text
nav2_controller::StoppedGoalChecker
```

在 `SimpleGoalChecker` 基础上额外检查机器人速度是否足够小。

```text
nav2_controller::LongitudinalGoalChecker
```

按目标点朝向把误差拆成：

```text
lon_err：纵向误差，前后误差
lat_err：横向误差，左右误差
yaw_err：航向角误差
```

适合精确停车、对齐停车姿态等场景。

```text
nav2_controller::PrecisionLongitudinalGoalChecker
```

高精度版本。它优先追求 `1cm` 精度，如果精调后仍无法进入 `1cm`，但一直保持在 `2cm` 内，则退让接受成功。

## 4. PrecisionLongitudinalGoalChecker

当前 `w1_nav2_params.yaml` 使用的是：

```yaml
goal_checker:
  plugin: "nav2_controller::PrecisionLongitudinalGoalChecker"
  desired_lon_goal_tolerance: 0.01
  desired_lat_goal_tolerance: 0.01
  accept_lon_goal_tolerance: 0.02
  accept_lat_goal_tolerance: 0.02
  yaw_goal_tolerance: 0.02
  enable_high_precision: false
  refine_cycles: 40
  stable_window_size: 5
  check_stopped: false
  trans_stopped_velocity: 0.01
  rot_stopped_velocity: 0.02
```

### 4.1 设计目标

```text
优先目标：
  纵向误差和横向误差都进入 1cm。

最终接受：
  如果进入 2cm 后继续精调一段时间仍进不了 1cm，
  但仍保持在 2cm 内，则允许导航成功。
```

这个策略可以称为：

```text
双阈值到点判定
```

或：

```text
分级容差退让策略
```

### 4.2 误差坐标系

误差按照目标点坐标系计算，不按照机器人当前 `base_link` 坐标系计算。

核心含义：

```text
以目标停车姿态为参考：
├── 机器人在目标前后偏多少：lon_err
└── 机器人在目标左右偏多少：lat_err
```

这样更适合停车和对齐目标姿态。否则如果机器人自身朝向已经偏了，用当前车体坐标系计算误差，会让纵向/横向误差的意义跟着车头一起变化。

### 4.3 状态逻辑

```text
PrecisionLongitudinalGoalChecker
├── 超出 2cm 接受区
│   ├── 清空精调状态
│   └── 返回 false
│
├── 进入 2cm 接受区，但没有进入 1cm 目标区
│   ├── 记录误差窗口
│   ├── refine_count 累加
│   └── 返回 false，继续让 controller 精调
│
├── enable_high_precision=false，且位置和角度满足接受精度
│   └── 直接返回 true，不再追求 1cm
│
├── 本次误差和近 10 次平均误差都进入 1cm 目标区
│   └── 返回 true
│
└── 精调达到 refine_cycles，且仍在 2cm 内
    └── 返回 true，退让接受成功
```

### 4.4 参数解释

```text
desired_lon_goal_tolerance / desired_lat_goal_tolerance
  优先追求的纵向/横向精度。当前是 0.01m，也就是 1cm。

accept_lon_goal_tolerance / accept_lat_goal_tolerance
  最终接受的纵向/横向精度。当前是 0.02m，也就是 2cm。

yaw_goal_tolerance
  目标航向角容差。当前是 0.02rad，约 1.15 度。

enable_high_precision
  是否启用高精度到点判定。
  true 表示先追求 desired_* 的 1cm 精度，再按 refine_cycles 退让到 accept_*。
  false 表示只要进入 accept_* 的 2cm 接受精度且 yaw 满足，就直接判定成功。

refine_cycles
  进入 2cm 后继续尝试精调的控制周期数。
  当前 controller_frequency=10Hz，refine_cycles=40 约等于 4s。

stable_window_size
  滑动误差窗口大小。
  当前 controller_frequency=10Hz，stable_window_size=5 约等于最近 0.5s。

check_stopped
  是否要求速度也足够小才算成功。
  当前为 false，表示只按位置和角度判断。

trans_stopped_velocity / rot_stopped_velocity
  check_stopped=true 时使用的线速度和角速度阈值。
```

## 5. near_goal_progress

`near_goal_progress` 是本仓库对近目标阶段的进展检查增强。

当前参数：

```yaml
near_goal_progress:
  enabled: True
  distance: 0.20
  required_movement_radius: 0.01
  required_movement_angle: 0.05
  movement_time_allowance: 2.0
```

含义：

```text
当机器人距离目标点小于 distance 时，进入近目标进展检查模式。

在近目标模式下：
  如果一段时间内移动距离或旋转角度都不足，
  controller_server 会认为 FollowPath 控制失败。
```

它的作用是防止机器人在目标附近长时间抖动、卡住或无效调整。

注意：

```text
如果 goal_checker 的精调时间变长，
near_goal_progress.movement_time_allowance 也需要匹配评估。
否则可能 goal_checker 还在尝试 1cm 精调，progress_checker 已经先判失败。
```

## 6. 修改插件时需要同步的文件

新增或删除 controller 相关插件时，通常需要同步修改：

```text
1. 插件头文件
   include/nav2_controller/plugins/<plugin_name>.hpp

2. 插件实现
   plugins/<plugin_name>.cpp

3. 编译配置
   CMakeLists.txt

4. pluginlib 注册
   plugins.xml

5. 参数文件
   nav2_bringup/params/*.yaml

6. 测试
   plugins/test/
```

不要只新增 `.cpp` 文件而忘记 `plugins.xml` 或 `CMakeLists.txt`，否则插件不会被编译或无法被 pluginlib 加载。

## 7. 常见日志与排查方向

### 7.1 goal_checker_id 不存在

如果日志中出现类似：

```text
requested goal_checker_id does not exist
```

优先检查：

```text
1. YAML 中 goal_checker.plugin 是否拼写正确
2. plugins.xml 是否注册了对应 class type
3. CMakeLists.txt 是否编译并安装了该插件库
4. 是否已经重新构建并 source 了新的 install/setup.bash
```

### 7.2 到目标附近但不成功

优先检查：

```text
1. lon_err / lat_err 是否进入 accept 容差
2. yaw_err 是否小于 yaw_goal_tolerance
3. refine_cycles 是否过长
4. near_goal_progress 是否先触发失败
5. 如果 check_stopped=true，速度阈值是否过严
```

### 7.3 很快成功但精度不够

优先检查：

```text
1. 是否真的加载了 PrecisionLongitudinalGoalChecker
2. desired_* 是否仍是 0.01
3. accept_* 是否被设置得过大
4. refine_cycles 是否过小
5. 日志中成功原因是 desired success 还是 accept fallback success
```

## 8. 测试与构建说明

本工作区有特殊约束：

```text
AI 代理禁止直接执行 colcon build。
```

用户手动构建时，可以参考：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select nav2_controller --parallel-workers 8
```

运行测试：

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select nav2_controller --event-handlers console_direct+
colcon test-result --verbose
```

如果只想看 goal checker 相关测试，可以关注：

```text
mobile_navigation/nav2_controller/plugins/test/goal_checker.cpp
```

## 9. 开发注意事项

1. `goal_checker` 只负责判断是否到达目标，不负责生成速度。
2. 如果要增强末端控制能力，应同时关注 MPPI 的末段 critic、速度限制和 goal_checker。单独改 goal_checker 只能延长精调时间，不能直接改变 MPPI 优化代价。
3. 精确停车建议使用目标点坐标系下的纵向/横向误差，不建议使用机器人当前车体坐标系作为误差参考。
4. 中文注释只写在关键函数、关键参数和关键分支上，不要逐行翻译代码。
5. 修改参数后要结合日志确认实际加载的是哪个插件，以及最终成功原因是什么。
