# PID 末端微调

本说明对应 `PIDController` 的 `ROTATING_TO_GOAL` 阶段。接近阶段仍保留规划零速时的越点倒退兜底；这里不增加路径重规划。

## 控制目标与执行顺序

横向尽量达到 10 mm，优先平稳完成。10 mm 是软优化目标，不替代当前 GoalChecker（到位检查器）的 2 cm 容差。控制器约 3 cm 的保持窗口只用于判断是否需要位置重试，不代表放宽最终到位精度。

```text
进入末端控制
├── LATERAL_REFINE：可选横向优化
│   ├── 软目标已满足、功能关闭或剩余行程不足 → 结束本轮横向优化
│   └── 剩余纵向行程足够 → 锁定一次不超过5°的切入方向，低速靠近目标
│       └── 软目标满足、行程不足或预测不再改善误差 → 结束本轮横向优化
├── LONGITUDINAL_TRIM：纵向微调
│   ├── 朝向未对齐或角运动未停稳 → 先制动，再对齐最终航向
│   ├── 预测终点符合到位窗口 → 低速前进/后退
│   └── 预测不合格，且角度已进入死区
│       ├── 当前位置合格 → 停止额外优化
│       └── 当前位置超差 → 停稳后对准目标点，进入位置重试
└── SETTLING：最终转向与停稳
    ├── 位置合格 → 保持最终航向，交给GoalChecker判定完成
    ├── 位置在保持窗口内持续超差 → 连续确认后重试
    └── 位置离开保持窗口 → 立即切换到位置重试
```

子阶段单向推进。10 mm 附近的定位噪声不会重新启动已结束的横向优化。位置重试回到合格范围后直接进入纵向微调，不再次追逐软目标。

横向优化只借用本来就需要的纵向位移。其可行性检查扣除当前实测速度对应的一周期响应距离与理想制动距离；线速度由纵向位置误差产生，不能因为横向尚有误差就持续驶过终点。切入方向在本轮优化开始时锁定，角命令继续使用已有变化率限制。

纵向微调使用当前实际航向预测移动到纵向死区附近后的横向与径向误差。平移门槛固定宽于角度死区；如果在死区内仍不能安全完成微调，就明确停稳或重试，不缩小航向门槛到控制器无法继续调整的范围。

正常停稳使用现有线减速度上限，方向反转先等待实测线运动停稳。外部速度上限优先于平滑制动；无效位置、速度或容差输入立即输出零速度。

## 参数

以下参数属于 `controller_server.PIDController`。本次维护的差速配置为 `nav2_bringup/params/w1_nav2_params_diff.yaml`。

| 参数 | 差速配置 | 语义与范围 |
| --- | --- | --- |
| `final_axis_lateral_tolerance` | 0.010 m | 横向软目标，有限且非负；0关闭横向优化，实际不超过当前横向到位容差 |
| `final_rotation_linear_deadband` | 0.005 m | 纵向微调死区，有限且大于0；实际不超过当前纵向到位容差的一半 |
| `final_rotation_max_v` | 0.05 m/s | 末端微调的线速度上限；仍受全局与外部速度上限约束 |
| `angular_error_deadband` | 0.008 rad | 普通角控制与最终转向共用的角度死区 |
| `retry_heading_tolerance` | 0.03 rad | 平移进入门槛，必须满足 `0 <= 角度死区 < 本值 <= 0.15` |
| `position_hold_hysteresis` | 0.01 m | 加在控制器到位容差上的保持余量，不改变GoalChecker验收标准 |
| `retry_position_capture_cycles` | 3 | 位置重试捕获，以及停稳后位置持续超差的连续周期门控 |

非法软目标和纵向死区分别恢复为 0.010 m、0.005 m；非法角度门控组合恢复为 0.008/0.03 rad，只在配置时警告。其他配置未显式设置软目标时同样使用代码默认值 0.010 m。

这些控制参数在 `configure()` 中读取，更改后需重新配置节点。运行中动态参数回调仍只处理 `verbose`。

## 状态、接口与CPU开销

- 状态只保存在PID插件实例中，不修改 `nav2_core::Controller` 或 GoalChecker 的公共接口。
- 新目标、取消/异常后的任务重置、`deactivate()` 和 `cleanup()` 清理末端相位、锁存方向和计数；同目标的 `setPlan()` / `setTrajectory()` 更新保留状态。
- `FinalRotationInput` 复用本周期已经算好的误差、实测速度与到位窗口判断；`decideFinalRotation()` 负责阶段决策，`computeFinalRotationCommand()` 负责调用已有PID。
- 新增决策只处理固定数量的栈上标量，无循环搜索、动态容器、堆分配、新锁或额外TF查询；不新增线程、定时器、Topic或周期性能采样。
- 正常新增日志仅在相位变化或进入重试时输出一次；无效输入警告按1秒节流。原有 `verbose` 周期诊断仍可关闭。

## 日志排查

日志使用英文固定字段，注释使用简短中文。`PID final phase` 记录阶段切换；`PID position retry` 记录重试来源、原因和所选航向；`PID final input invalid` 列出无效输入并说明输出归零。参数警告包含参数名、实际值、约束和回退值。

| 字段或原因 | 含义 |
| --- | --- |
| `reason=align_cut_in` / `align_goal` | 等待对齐切入航向 / 最终航向及角速度停稳；线运动未停稳时先制动 |
| `reason=lateral_refine` / `longitudinal_trim` | 正在执行横向优化 / 纵向微调 |
| `reason=reverse_brake` / `retry_brake` | 换向前 / 位置重试前制动 |
| `reason=trim_complete` / `trim_skipped` | 纵向微调完成 / 位置已合格，放弃不可行微调 |
| `reason=settling` / `position_miss_pending` | 最终转向与停稳 / 正在累计位置超差周期 |
| `reason=hold_window_exit` / `position_miss` | 离开保持窗口 / 停稳后位置持续超差，进入重试 |
| `reason=point_retry` | 角度已在死区内，但预测位置仍超差，改为对准目标点 |
| `reason=terminal_speed_zero` / `lateral_miss` | 终端规划速度过低仍未到位 / 保持最终航向的重试出现连续横向超差 |
| `refine_exit=soft_target_met` / `travel_short` | 横向已达软目标 / 扣除制动余量后行程不足 |
| `refine_exit=disabled` / `direction_changed` / `prediction_unsafe` | 横向优化关闭 / 修正方向改变 / 预测不再改善误差或越出保持窗口 |

`lon_m`、`lat_m` 是固定目标坐标系的位置误差，`tol_m` 按纵向、横向排列，`hold_margin_m` 是保持余量。`yaw_rad` 单位为 rad，`vx_mps` 为 m/s，`wz_rps` 为 rad/s。周期日志中 `base_err` 属于机体坐标系，`goal_err` 属于固定目标坐标系，角度注明 `deg`。两种坐标系的误差不能直接混用。

周期诊断复用原有 100 ms 节流日志，补充 `reason`；`none` 表示本周期未执行末端决策或未退出横向优化。设置 `PIDController.verbose=0` 可关闭周期 INFO 诊断，阶段事件和异常警告仍保留；DEBUG 诊断由日志级别独立控制。制动与无效输入分支清理位置环诊断值，避免显示上一周期数据。

## 验证与限制

`test_pid_speed_loop.cpp` 的 `FinalRotation` 测试组覆盖死区静止反例、无纵向行程、横向优化方向对称、行程耗尽、制动余量、重试方向、连续周期门控、速度反转、参数与无效输入、同目标更新和生命周期重置。

AI代理不执行该工作区构建。用户手动构建后，可在已加载ROS与工作区环境的终端执行：

```bash
./build/nav2_pid_controller/test_pid_speed_loop --gtest_filter='FinalRotation.*:PidTaskReset.*:ApproachStage.*'
```

实机需覆盖前进/后退、顺逆时针、目标航向跨±π、定位噪声和多轮重复导航，并对比相同路线、相同 `verbose` 设置下的控制周期耗时与CPU占用。本轮代码检查不能替代实机性能数据。

几何预测不包含完整底盘动力学，理想制动距离也不代表实机停车距离保证。空间不足时允许保留10～20 mm横向误差，避免为软目标额外折返。GoalChecker保持独立到位判定，因此不保证每次都完成10 mm优化。

需要关闭横向优化时，将 `final_axis_lateral_tolerance` 设为0并重新配置；纵向微调、位置重试和越点倒退能力仍保留。恢复本轮修改前的完整行为时，应回退本轮工作区差异，并保留用户原有暂存修改。
