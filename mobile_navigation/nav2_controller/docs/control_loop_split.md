# FollowPath：10 Hz 局部轨迹规划、25 Hz 控制

## 当前实现

FollowPath 只有快照链路，原串行规划/控制入口和 runPlanningPipeline 已移除。
频率由 controller_server 的 planning_frequency=10.0、controller_frequency=25.0 配置。
不再根据 FollowPath split_control 字段切换链路；现有 Action/BT 消息未新增必填字段。

controller server 激活时创建一个持久规划线程；无 Action 时条件变量等待。
每个 Action 生成新的 session；路径、限速、传感器失效改变 revision。
取消/结束立即撤销请求和结果；生命周期停用/析构在 join 前向规划插件发出
requestCancel() 协作式取消请求。插件应尽快返回；旧调用结束后仍会比较版本并丢弃迟到结果。

## 输入输出与所有权

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| FollowPath goal.path | nav_msgs/Path | 全局几何路径输入 |
| 现有里程计订阅、TF | Odometry / TF | 每拍最新位姿和速度 |
| speed_limit | nav2_msgs/SpeedLimit | 限速事件，分别由规划/控制所属线程消费 |
| local_trajectory | nav_interfaces/RobotTrajectory | 控制侧接受的完整局部轨迹 |
| cmd_vel | geometry_msgs/Twist | 每个控制周期重算的速度；沿用 launch remapping |
| cmd_vel_hard_stop | std_msgs/Empty | 失效时通知下游硬停车，同时发送零速 |

局部规划流水线包含 LocalPlanner 几何规划和 SpeedPlanner 速度规划，两者都只在规划线程调用。
最终输出 RobotTrajectory，points 含 pose、longitudinal_velocity、angular_velocity、acceleration、relative_time。
plan_id 记录 session:revision。控制只接受非空、有效轨迹，不再在未配置 SpeedPlanner 时回退几何路径跟踪。
未实现 Controller::setTrajectory 的插件现在抛错，避免静默忽略速度剖面；应重编相关控制器插件。

规划线程不修改控制器插件、current_trajectory_ 或控制阶段状态。它读取不可变请求，完成后在短锁内交接完整结果。
全局路径仅在 path revision 改变时复制为共享不可变对象；25 Hz 控制周期只复制小型请求状态，
不再重复复制完整 Path，也不再为 Request 单独进行堆分配。
计算下一轮时不清空上一成功结果。控制侧只在接受新结果时分发轨迹，每拍使用当前位姿和里程计重算速度。
失败结果立即使旧轨迹不可用。源时间、版本及有效性检查不等于物理碰撞安全证明。

规划边界使用最新实测速度，控制侧已消费目标的有效加速度反馈通过请求交接。
末端制动锁存由规划线程持有，跨同任务规划保留，新 Action 重置。
新实现没有复用旧串行流水线的完整前缀拼接算法；不能把这次改造视为其运动连续性的等价验证。

## 停车条件

- costmap 非 current、机器人位姿获取失败。
- 首次轨迹未完成、规划失败、版本不一致、轨迹超过 planning_result_max_age（默认 0.3 秒）。
- Action 取消或控制器报告碰撞。

每个 25 Hz 控制周期检查一次，速度发布前再次核对 costmap、已消费轨迹年龄及失效版本，阻止计算期间发生故障后继续输出非零。
失效后停用请求并作废轨迹，恢复后必须取得新版本规划才允许动作。
Controller Server 不再直接订阅或强制要求四路 LaserScan；传感器断流安全由 costmap current、
Collision Monitor、底盘急停及下游命令超时负责。

## 配置与验证

w1_nav2_params_diff.yaml 已设 10/25 Hz，速度规划器 control_dt 为 0.04 秒。
MPPI 的 model_dt 保留原预测离散间隔，未为了控制频率改变而缩短预测范围。
这些频率和健康阈值需在非活动状态重配置；运行时修改会明确拒绝，不会返回成功但静默无效。

测试：snapshot_validation_test 覆盖版本、成功状态和轨迹年龄；snapshot_worker_test 使用阻塞假规划器验证持久线程、旧快照复用、协作式取消、任务隔离与失败传播。
测试不证明整机 10/25 Hz 实时截止时间、物理刹停距离或轨迹连续性；本次只修改本地，尚未部署机器人。

### 本地验证记录

Release 构建通过；6 个 gtest 目标全部通过（snapshot_validation_test、snapshot_worker_test、pctest、gctest、test_dynamic_parameters、test_local_planning_stage）。使用 ROS_DOMAIN_ID=124，与机器人域隔离。
构建目录：/tmp/nav_snapshot_build；测试输出：/tmp/nav_snapshot_verified_tests.log。
