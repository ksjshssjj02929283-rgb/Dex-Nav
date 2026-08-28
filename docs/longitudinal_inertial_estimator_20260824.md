# 独立纵向状态估计器（2026-08-24）

## 结论

纵向惯性估计已经从 `nav2_controller` 完整拆出，成为独立 ROS 2 包
`nav2_longitudinal_state_estimator`。Controller 不再订阅 IMU、计算 odom 加速度或维护
EKF，只订阅 `/navigation/longitudinal_state`，并在送入 SpeedPlanner 前执行最后一道物理
边界整形。

旧的 `[v, a, bias]` EKF、odom 回归估计器、固定权重 IMU 融合器和
`AccelerationStateObserver` 的源码、参数、成员和测试均已删除，不存在串联叠加。

## 数据流与职责

```text
/feedback/odom -----------+
                          +--> nav2_longitudinal_state_estimator
/imu/chassis/refined -----+       |  标定、延迟融合、IMU重放、bias静止学习
                                  v
                    /navigation/longitudinal_state
                                  |
                                  v
                        nav2_controller
                        |  新鲜度检查
                        |  跨周期负jerk边界整形
                        v
                        SpeedPlanner (v0, a0)
```

包之间的边界：

- `nav2_longitudinal_state_estimator` 负责传感器时间戳、状态估计、滤波、有效性和诊断；
- `nav2_controller` 负责规划边界安全，不再拥有任何传感器估计算法；
- `nav2_speed_planner` 只消费 Controller 给出的 `InitialMotionState`。

S型速度规划还包含显式精确控制器交接边界：

```text
goal前0.30m：速度必须不高于0.25m/s
goal前0.05m：继续下降到0.05m/s蠕行速度
最终到点：PIDController/GoalChecker归零
```

0.30m位置会被插入为精确路径点，0.25m/s作为速度包络上界参与Ruckig求解，因此减速
从更早位置按jerk连续地产生，不是在RPP切换PID的一拍硬钳速度。

### `/imu/chassis/refined` 坐标语义

robot_server 已经用初始化得到的固定旋转，将 acceleration 和 angular velocity 变换到
静态水平对齐坐标系，但消息保留了原 `imu_chassis_link` 的 frame_id 和 orientation。
因此新估计器对该话题采用以下明确约定：

```text
imu.input_is_level_aligned = true
第一帧orientation建立静态水平参考x轴
后续orientation重力向量先投影到该水平参考系
只减去与refined acceleration同坐标系的gravity_x
```

这避免直接用原机体系 orientation 对已变换的 acceleration 做不在同一坐标系的重力
投影，同时仍能补偿机器人相对启动姿态发生的动态俯仰。每次 robot_server 初始化后
剩余的纵向常值误差由 `[v,bias]` 估计器在静止阶段学习，不再用固定的
`-0.11044 m/s²` 抵消坐标误差。

若未来直接改订阅原始机体系 `/imu/chassis`，必须显式设置
`imu.input_is_level_aligned=false`，此时才会启用 orientation 重力投影，并需重新标定。

## 估计模型

状态为：

```text
x = [纵向预测速度 v, 标定后残余加速度 bias]
```

IMU 是预测输入，odom 是速度观测：

```text
a_current = calibrated_imu_a - bias
v(k+1) = v(k) + a_current * dt
z_odom = v
```

滤波器在 `now - fusion_delay` 的延迟时间线上修正 odom，然后复制状态并重放该时刻
之后缓存的 IMU，输出当前时刻预测。这是一维化的 PX4“延迟融合 + 输出预测”结构。

规划器的 `v0` 仍使用最新 odom 速度；预测速度只用于诊断和一致性保护，不会直接把
机器人目标速度推高。预测与 odom 相差超过 `hard_velocity_reanchor` 时会立即重锚，
避免统计门限连续拒绝后锁死。

## bias 学习

bias 只在以下条件持续满足 `stationary_hold_duration` 后学习：

```text
|odom v| <= stationary_max_abs_velocity
|odom wz| <= stationary_max_abs_wz
|imu a - bias| <= stationary_max_abs_acceleration
```

机器人运动时 bias 不吸收真实加减速，只保留极小过程噪声。

## Controller 边界整形

估计器发布原始和低通加速度，Controller 再生成真正交给 SpeedPlanner 的 `a0`。
正常前进的 TRACK 段保留上周期Ruckig规划加速度，并用实测做有限纠偏：

```text
innovation = clamp(measured_a - planned_a, -innovation_limit, innovation_limit)
corrected_plan = planned_a + gain * innovation
a0 = max(measured_a, corrected_plan)
```

前进时数值较大的加速度意味着更长停车距离，因此最后的 `max` 是安全约束：不会把
底盘尚未建立的规划减速度当成真实制动力。以下情况不保留规划加速度，直接使用实测：

- 终点制动已经锁存；
- APPROACH、ALIGN、PREPARE等非TRACK阶段；
- 倒车；
- 估计器或规划连续性无效。

此外：

- 更大的加速度立即采用，因为它会增加停车距离，必须保守响应；
- 更强的负加速度按 `conditioner.max_negative_jerk` 跨周期逐步采用；
- 状态无效或超龄时退回既有规划连续性，不再启动任何旧估计链。

可通过 `initial_motion.planned_correction.enabled=false` 回退到纯实测边界。规划纠偏的
代价是：正加速度会比纯实测方案建立得更快；若增益过低会忽略真实跟踪不足，过高则
重新退化为慢速纯实测。当前 `gain=0.40`、`innovation_limit=0.30m/s²` 与既有实车对比
版本一致，且终点制动不使用该融合。

## 参数归属

估计器参数位于顶层节点 `longitudinal_state_estimator.ros__parameters`：

| IMU前端参数 | 当前值 | 作用 |
|---|---:|---|
| `imu.input_is_level_aligned` | true | 用首帧姿态把原orientation重力变换到refined水平参考系 |
| `imu.static_bias` | 0.0 m/s² | 可选硬件常值；W1运行时偏置交给状态bias学习 |
| `imu.scale_to_odom` | 0.767 | refined纵向加速度到odom尺度 |
| `imu.imu_x_offset` | 0.273 m | 原地转动的向心加速度补偿 |

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `estimator.enabled` | true | W1 参数中启用；程序默认 false |
| `estimator.fusion_delay` | 0.03 s | 延迟融合时间线 |
| `estimator.buffer_duration` | 0.50 s | IMU/odom 缓存长度 |
| `estimator.max_age` | 0.10 s | 输入最大年龄 |
| `estimator.reset_timeout` | 0.50 s | 时间断档重置阈值 |
| `estimator.imu_acceleration_noise` | 0.10 m/s² | IMU 积分过程噪声 |
| `estimator.bias_process_noise` | 0.0005 m/s²/√s | bias 极慢随机游走 |
| `estimator.odom_velocity_noise` | 0.02 m/s | odom 速度观测噪声 |
| `estimator.odom_innovation_gate` | 5 sigma | odom 创新裁剪门限 |
| `estimator.hard_velocity_reanchor` | 0.08 m/s | 速度硬重锚阈值 |
| `estimator.stationary_hold_duration` | 0.30 s | bias 学习前静止时间 |
| `estimator.acceleration_filter_tau` | 0.05 s | 输出加速度低通常数 |
| `estimator.turning_hold_max_abs_wz` | 0.15 rad/s | 转弯时抑制 body-x IMU |
| `estimator.turning_decay_tau` | 0.20 s | 转弯时向零衰减常数 |

Controller 只保留以下接口参数：

```yaml
initial_motion:
  longitudinal_state_topic: /navigation/longitudinal_state
  longitudinal_state_max_age: 0.10
  conditioner:
    min_acceleration: -0.40
    max_acceleration: 0.50
    max_negative_jerk: 0.50
```

## 生命周期与话题

`navigation_launch.py` 会在 `controller_server` 前启动并激活
`longitudinal_state_estimator`，独立进程和 component composition 两种模式均已接入。
节点继承 `nav2_util::LifecycleNode`，激活时创建 Nav2 bond，停用时销毁 bond；因此
生命周期管理器可以在确认估计器心跳后继续激活 ControllerServer 和后续导航节点。

输出话题：

- `/navigation/longitudinal_state`：带有效标志、状态、协方差、时延和计数器的结构化消息；
- `/navigation/longitudinal_state_diagnostics`：标准 ROS diagnostics。

## 部署文件

- 整个 `mobile_navigation/nav2_longitudinal_state_estimator/`；
- `mobile_navigation/nav2_controller/` 中本次有差异的源码、构建文件和测试脚本；
- `mobile_navigation/nav2_bringup/launch/navigation_launch.py`；
- `mobile_navigation/nav2_bringup/params/w1_nav2_params_diff.yaml`；
- `mobile_navigation/nav2_bringup/package.xml`；
- `mobile_navigation/navigation2/package.xml`。

应先编译新包，再编译依赖它的 Controller 和 bringup。回退时必须成套恢复 Controller、
bringup 和参数文件；仅停掉新节点会让 Controller 安全退回规划连续性，但不等价于恢复
旧实现。
