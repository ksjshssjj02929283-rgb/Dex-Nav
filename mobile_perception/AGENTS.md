# mobile_perception 工作总结

本文档用于快速帮助后续开发者理解 `mobile_perception` 包当前职责、关键节点、`mobile_control_node.cpp` 的控制逻辑，以及最近一次关于“直线末端姿态对齐”的修改背景。

## 目录概览

- `src/mobile_control_node.cpp`
  低层精确控制节点。订阅目标位姿，结合 TF 与 odom 反馈输出 `control/cmd_vel`。
- `src/apriltag_detection_node.cpp`
  AprilTag 感知相关节点。
- `src/charging_pose_service.cpp`
  充电位姿相关服务。
- `config/mobile_control_params.yaml`
  `mobile_control_node` 的主要参数配置来源。
- `launch/mobile_control.launch.py`
  控制节点启动入口之一。
- `scripts/`
  包含若干测试、导航串联、控制辅助脚本。

## mobile_control_node 整体职责

`mobile_control_node` 的主要作用是接收 `control/ctrl_pose` 目标，并在机器人底盘坐标系下输出平滑速度控制命令到 `control/cmd_vel`。

当前节点不是单一控制器，而是一个带模式切换的控制状态机：

- `GoalMode::POSE`
  普通位姿控制。既关心位置，也关心终态朝向。
- `GoalMode::STRAIGHT`
  直线靠近目标。主要用于横向偏差和角度偏差都较小的情况，尽量保持直行。
- `GoalMode::ROTATE`
  纯旋转模式。目标平移误差很小，仅做原地转向。

此外还有一个末端子状态：

- `TerminalState::STRAIGHT_YAW_ALIGN`
  直线模式到点后，如果位置已满足但朝向还差一点，则进入受限的末端姿态对齐。

## 当前控制主流程

### 1. 输入与状态更新

- 目标由 `control/ctrl_pose` 输入。
- 目标若不是 `map` 坐标系，会先被转换到 `map`。
- odom 由 `feedback/odom` 输入。
- `pre_cmd_vel_` 现在表示“底盘实际反馈速度”，不再表示上一拍发布命令。

这一点很关键：当前的加速度约束、平滑停车、终态旋转约束，都是基于 odom 实际速度做的，而不是基于期望命令做的。

### 2. 模式判定

在 `targetCallback()` 中，收到目标后会按输入目标在局部坐标中的特征选择控制模式：

- 若平移距离很小但航向误差较大，进入 `ROTATE`
- 若横向偏差小且角度偏差小，进入 `STRAIGHT`
- 其他情况进入 `POSE`

对应参数：

- `goal_mode_linear_yaw_threshold`
- `goal_mode_linear_lateral_threshold`
- `goal_mode_rotate_xy_threshold`

### 3. 正常控制分支

在 `controlLoop()` 中：

- 若处于取消控制状态，执行基于 odom 的平滑减速
- 否则将 `map` 下目标变换到 `base_link`
- 计算 `dist` 和 `yaw_error`
- 根据是否倒车适合，选择前进或后退控制

常规控制器包括：

- `calculateNoReverseGracefulVelocity()`
  前向 rho-alpha-beta 变体控制
- `calculateReverseGracefulVelocity()`
  后向倒车控制
- `computeRotateToHeadingCommand()`
  纯旋转控制

然后统一再经过：

- 线速度最小有效值修正 `applyMinimumLinearSpeed()`
- 基于 odom 的线角加速度约束
- 输出低通滤波 `filtered_cmd_vel_`

### 4. 到点判定

- `STRAIGHT` 模式下，优先看 `xy_tolerance_`
- 其他模式通常要求 `dist < xy_tolerance_` 且 `yaw_error < yaw_tolerance_`
- 进入 `is_goal_` 后，会持续发布若干拍零速度，保证平稳结束

## 直线模式的特殊处理

当前直线模式有两层特殊约束：

### 1. 近目标角速度限制

在 `goal_mode_ == STRAIGHT && dist < near_target_dist_` 时，会额外限制：

- `cmd_vel.angular.z` 被夹到 `[-straight_near_goal_angular_max_, straight_near_goal_angular_max_]`

目的：

- 防止接近目标时边走边转过猛
- 让直线靠近行为更稳定，不在近端大幅摆头

### 2. 到点后的末端姿态对齐

若直线模式下：

- 已满足 `dist < xy_tolerance_`
- 且 `straight_terminal_yaw_align_enable_ == true`
- 且 `fabs(yaw_error) > straight_terminal_yaw_trigger_`

则不会立刻宣布完成，而是进入：

- `TerminalState::STRAIGHT_YAW_ALIGN`

进入该状态后：

- 线速度强制为 0
- 只做小角速度姿态修正
- 姿态收敛后再置 `is_goal_ = true`

## 最近一次修改总结：直线末端姿态对齐控制器已独立

### 原始问题

之前的 `computeStraightTerminalRotateCommand()` 复用了 `computeRotateToHeadingCommand()`。

但 `computeRotateToHeadingCommand()` 内部自带停止条件：

- `abs_error < rotate_stop_yaw_error_` 时直接输出 `angular.z = 0`

而 `computeStraightTerminalRotateCommand()` 外层还试图加一层：

- 若角速度非零且小于 `min_angular_vel_`，则强制给最小角速度

这会导致一个逻辑冲突：

- 一旦内层旋转控制器已经因为小误差输出了 0
- 外层“最小角速度补偿”根本不会生效

因此之前末端直线对齐里那层最小角速度逻辑，在大多数参数组合下是没有意义的。

### 当前修改后的行为

现在 `computeStraightTerminalRotateCommand()` 已改成独立控制器，不再调用通用的 `computeRotateToHeadingCommand()`。

它当前只受以下参数约束：

- `straight_terminal_rotate_stop_yaw`
- `straight_terminal_rotate_max_w`
- `rotate_to_heading_kp`
- `rotate_to_heading_max_angular_accel`
- `min_angular_vel`

当前逻辑为：

1. 若 `fabs(yaw_error) <= straight_terminal_rotate_stop_yaw_`，直接输出 0
2. 否则做 `rotate_to_heading_kp_ * yaw_error` 的 P 控制
3. 角速度限制到 `straight_terminal_rotate_max_w_`
4. 再根据 `rotate_to_heading_max_angular_accel_` 做可实现角加速度约束
5. 再根据制动距离 `sqrt(2*a*error)` 约束可停车最大角速度
6. 若结果非零但过小，则提升到 `min_angular_vel_`

这样修改后：

- 末端姿态对齐有独立停止阈值
- `min_angular_vel_` 在末端对齐场景下重新变得有效
- 通用原地旋转控制与“直线到点后的轻柔补角”被解耦

## 当前关键参数关系

后续调参时，优先关注下面几组关系。

### 1. 直线末端进入阈值与退出阈值

- `straight_terminal_yaw_trigger`
- `straight_terminal_rotate_stop_yaw`

建议：

- `straight_terminal_yaw_trigger > straight_terminal_rotate_stop_yaw`

否则容易出现：

- 刚进入末端姿态对齐就满足退出条件
- 或者进入/退出边界过近，行为不明显

### 2. 最小角速度与停止阈值

- `min_angular_vel`
- `straight_terminal_rotate_stop_yaw`
- `rotate_to_heading_kp`

如果：

- `min_angular_vel` 过大
- `straight_terminal_rotate_stop_yaw` 过小

则末端可能在接近停止区间时来回轻微摆动。

### 3. 末端最大角速度

- `straight_terminal_rotate_max_w`

这个值过大时：

- 末端补角会显得突兀
- 甚至影响“直线停稳后再微调”的观感

这个值过小时：

- 收敛时间会变长
- 更容易频繁打出“still not converged”日志

### 4. 近目标直行角速度限制

- `straight_near_goal_angular_max`

这个参数影响的是“接近目标时边走边转”的能力，不是停下后的末端姿态对齐能力。两者不要混为一谈。

## 后续排查建议

如果后续发现“直线到点后朝向不准”或“末端原地补角抖动”，建议按下面顺序排查：

1. 看是否真的进入了 `STRAIGHT_YAW_ALIGN`
2. 看进入时 `yaw_error` 是否明显大于 `straight_terminal_yaw_trigger`
3. 看 `filtered_cmd_vel_.angular.z` 是否因为滤波过强而收敛太慢
4. 看 `min_angular_vel` 是否过大，导致小误差区间抖动
5. 看 odom 反馈的 `angular.z` 是否太噪，导致加速度约束判断失真
6. 看 `straight_terminal_rotate_max_w` 是否过小，导致补角很慢

## 已知实现特点与注意事项

- `pre_cmd_vel_` 当前来自 odom，而不是上一拍控制输出
- 控制链路里同时存在“基于实际速度的加速度约束”和“输出低通滤波”
- 末端对齐状态中仍然对角速度做滤波，所以实际停转会比理论控制输出更柔和
- `straight_terminal_rotate_max_cycles` 当前主要用于周期性打印未收敛告警，不会强制跳出末端对齐
- `computeRotateToHeadingCommand()` 仍保留通用旋转模式使用，不应再拿它替代末端直线姿态对齐逻辑

## 建议的续接入口

如果后续继续优化本模块，优先从这些位置入手：

- `src/mobile_control_node.cpp`
  关注 `targetCallback()`、`computeRotateToHeadingCommand()`、`computeStraightTerminalRotateCommand()`、`controlLoop()`
- `config/mobile_control_params.yaml`
  把本文件中的阈值关系同步到实际运行参数
- 实机或仿真日志
  重点观测：
  `yaw_error`、`filtered_cmd_vel_.angular.z`、`pre_cmd_vel_.angular.z`

## 本次会话结论

- 已确认：旧版 `computeStraightTerminalRotateCommand()` 中的最小角速度逻辑会被通用旋转控制器的内层停转阈值覆盖，实际意义很弱
- 已修改：末端直线姿态对齐控制器改为独立实现
- 已保留：通用原地旋转控制器 `computeRotateToHeadingCommand()` 继续服务于非直线终态场景
- 已修复：该函数附近一处日志格式写法错误
