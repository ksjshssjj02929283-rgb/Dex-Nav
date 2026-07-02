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
│   ├── nav2_simple_goal_checker.cpp
│   ├── nav2_stopped_goal_checker.cpp
│   ├── nav2_longitudinal_goal_checker.cpp
│   └── precision_nav2_longitudinal_goal_checker.cpp
│
├── include/nav2_controller/
│   ├── controller_server.hpp
│   └── plugins/
│       ├── simple_progress_checker.hpp
│       ├── pose_progress_checker.hpp
│       ├── nav2_simple_goal_checker.hpp
│       ├── nav2_stopped_goal_checker.hpp
│       ├── nav2_longitudinal_goal_checker.hpp
│       └── precision_nav2_longitudinal_goal_checker.hpp
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

`nav2_controller` 不再导出 goal checker 插件。Goal checker 已统一归口到
`nav2_modules/goal_checkers/`，运行配置应使用这些插件类型：

```text
nav2_simple_goal_checker::SimpleGoalChecker
nav2_stopped_goal_checker::StoppedGoalChecker
nav2_position_goal_checker::PositionGoalChecker
nav2_rotation_goal_checker::RotationGoalChecker
nav2_longitudinal_goal_checker::LongitudinalGoalChecker
nav2_high_precision_goal_checker::HighPrecisionGoalChecker
```

`nav2_controller` 只保留 progress checker 和 ControllerServer。

## 4. near_goal_progress

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
1. lon_err / lat_err 是否进入配置的 lon_goal_tolerance / lat_goal_tolerance
2. yaw_err 是否小于 yaw_goal_tolerance
3. near_goal_progress 是否先触发失败
4. 如果使用 stopped/high_precision goal checker，速度阈值是否过严
```

### 7.3 很快成功但精度不够

优先检查：

```text
1. 是否真的加载了预期的 nav2_modules/goal_checkers 插件
2. lon_goal_tolerance / lat_goal_tolerance / xy_goal_tolerance 是否设置过大
3. stateful=true 时是否在第一次进入位置窗口后跳过后续位置检查
4. action 请求中是否传入了非预期 goal_checker_id
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
