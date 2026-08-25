# diff-sim-container — W1 双轮差速底盘仿真

与 `swerve-sim-container`（四轮全向）平行的差速仿真容器。机器人模型为 **W1 实机差速底盘**
（双驱动轮 + 前后激光 + IMU），**自包含**：URDF 快照与 Visual meshes 已内置，
不依赖 dexe_mobile_application，后续只维护 mobile_simulation 内的内容。

## 结构

```
diff-sim-container/
  src/
    w1_diff_gazebo/
      urdf/source/DexforceW1V020.urdf   # W1 可视化 URDF 快照（源）
      meshes/Visual/                    # 视觉 mesh（内置）
      config/real_robot_default_tf.json # 真机 bag 提取的 64 条默认机器人内部 TF
      scripts/gen_w1_diff_sim_urdf.py   # 生成器：可视化 URDF → 仿真 URDF（模型更新后重跑）
      urdf/w1_diff_sim.urdf             # 生成产物（已提交，mesh URI 指向本包）
      src/sim_cmd_vel_bridge.cpp        # 生产 Nav2 速度输出 → Gazebo 底盘话题适配
      launch/spawn_w1_diff.launch.py    # rsp + spawn + sim_cmd_vel_bridge
```

## 运行

一键入口：`ros2 launch w1_nav2 diff_sim.launch.py`（gzserver/gzclient + W1 + Nav2 + Rviz）。
传感器及其他仿真选项统一在 `w1_nav2/config/diff_sim.yaml` 中配置；启动命令后不再追加
独立的传感器参数。Gazebo 会在生成机器人实例时移除已关闭的传感器插件，关闭 RGBD
也会同步停止对应点云转换节点，不需要维护不同传感器组合的 URDF 副本。
全向对应 `ros2 launch w1_nav2 omni_sim.launch.py`。
Nav2 参数：直接复用生产配置 `nav2_bringup/params/w1_nav2_params_diff.yaml`。
导航模式的静态 `map→odom` 由 `nav2_bringup` 唯一发布；建图模式由 SLAM 发布。

## W1 模型更新同步

```bash
G=<workspace>/dexe_mobile_nav/mobile_simulation/diff-sim-container/src/w1_diff_gazebo
cp <新版>/urdf/DexforceW1V020.urdf "$G/urdf/source/"
rsync -a --delete <新版>/meshes/Visual/ "$G/meshes/Visual/"
cd "$G" && python3 scripts/gen_w1_diff_sim_urdf.py "" urdf/w1_diff_sim.urdf
cd <workspace> && colcon build --packages-select w1_diff_gazebo --parallel-workers 8
```

`real_robot_default_tf.json` 是独立姿态基线，更新可视化 URDF 时不要覆盖。生成器要求其
64 条机器人内部边与 URDF 拓扑完全一致，缺边或多边会直接失败，避免模型更新后静默退回零位姿态。

## 真机默认 TF 姿态

- 基线来自 `/home/fengbro/下载/rosbags/mapping` 的 `/tf`、`/tf_static` 首帧。
- bag 中腰、头、双臂和手指的 40 条运动关节 TF 在 2991 帧内保持不变；仿真将这些实际姿态固化为 fixed，避免无控制器上身在 Gazebo 中倒塌。
- `odom→base_footprint` 在录包期间变化约 19.7m，属于行驶状态，不是机器人默认姿态，因此仍由 Gazebo 差速插件动态发布。
- 前后雷达的 ROS TF 使用真机逻辑位姿；Gazebo ray sensor 用 link 内部 `<pose>` 保留原物理扫描高度，防止激光扫到底盘碰撞盒。

## 验证基线（2026-07-21）

- profile→生成 URDF：64/64 边一致，最大平移误差 0、最大旋转误差 `2.98e-8 rad`。
- 独立 ROS domain 无 GUI Gazebo：模型成功 spawn，`/feedback/odom` 数值有限。
- `base_link→laser_front_link` 实测 `xyz=(0.166,0.137,0)`、`yaw=0.790`；`torso_base_link→ankle` 实测 `xyz=(0,0,0.292)`、`RPY=(1.571,0.710,0)`，均与 bag 一致。
- 空世界前雷达 720 束全部为 `inf`，`<0.6m` 自扫命中 0。Gazebo/Rviz 视觉与完整 Nav2 回归仍需人工确认。

## 话题契约

| 话题/TF | 方向 | 说明 |
|---------|------|------|
| `/control/cmd_vel_nav` | 订阅 | 生产 `velocity_smoother` 输出；由 `sim_cmd_vel_bridge` 原样转发 |
| `/control/cmd_vel` | 订阅 | 差速插件底盘速度指令，也是 teleop/直驱接口 |
| `/feedback/odom` + `odom→base_footprint` TF | 发布 | ground-truth 里程计（话题名对齐实机 chassis 反馈）|
| `/laser_scan/front`、`/laser_scan/rear` | 发布 | 前后激光（±135°，15Hz）|
| `/imu/chassis` | 发布 | IMU（100Hz）|
| 机器人内部 TF | 发布 | 64 条父子变换对齐真机默认 bag；轮 TF 随 joint_states 动态更新 |

差速仿真直接复用生产参数，因此不在 YAML 中改写速度输出话题。仿真适配层负责
`/control/cmd_vel_nav → /control/cmd_vel`；若出现“已有全局路径但 odom 始终为零”，
应先确认 `/sim_cmd_vel_bridge` 存在，并检查这两个话题的发布者/订阅者。

生产参数默认使用 `RouteGridPlanner`，并配置
`route_fallback_planner: "ThetaStarBased"`。当
`/map_asset_manager/get_route_graph` 不存在且没有已加载路网时，Route planner 返回
`NO_ROUTE_GRAPH`，PlannerServer 立即转交 Theta Star；后台 RouteGraph 服务发现日志
不会阻塞本次规划。后续局部跟踪仍由当前 controller（例如 MPPI）执行。

## 关键参数与物理简化（详见生成器 docstring）

- wheel_separation = 0.5078 / wheel_radius = 0.063（取自 URDF 几何链）
- 上身 40 关节按真机默认姿态固定；碰撞体 = 2 轮圆柱 + 底盘盒（0.70×0.50×0.14，顶面低于 ray sensor 物理扫描平面，防自扫）+ 前后中心线 2 个无摩擦万向球
- 左右驱动轮与前后中心线万向球底面共面，构成菱形四点支撑；避免四角 skid 共面时六点接触卸载驱动轮，也避免抬高 skid 后底盘绕轮轴产生静态俯仰
- 惯量 = 2 轮原值 + 底盘 60kg 集中盒惯量（可视化惯量直接进物理引擎会 NaN）
