# nav2_nav_monitor — 导航任务级监控

以 `/navigate_to_pose` 的 goal 生命周期为任务边界，每次导航任务结束自动落盘一份
任务级 JSON（摘要 + 轨迹 + 双速度采样 + 阶段切换序列），配套脚本离线出图回溯。

## 数据流

```
bt_navigator ── goalReceived() 回显 ──→ /nav_monitor/goal_pose_echo (+behavior_tree)
controller_server ── setPlannerPath() 回显 ──→ /nav_monitor/current_global_path
controller_server ── setControllerPhase() 回显 ──→ /nav_monitor/controller_phase
controller_server ── 每控制周期回显 ──→ /nav_monitor/desired_velocity（最近规划状态）
                                      → /nav_monitor/controller_target（控制器目标 v/a）
/navigate_to_pose/_action/status + feedback（标准 action 内省，零改动）
cmd_vel_nav（控制器输出，平滑前）+ control/cmd_vel_nav（velocity_smoother 平滑后）
odom（实测反馈，w1 配置为 feedback/odom）
                    ↓ 全部被 nav_monitor 节点被动订阅
  ~/.dexe/navigation/nav_monitor/nav_task_<时间戳>_<结果>.json（一任务一文件）
```

## 启用

三处开关（三份 bringup YAML 均已默认启用）：

| 节点 | 参数 | 作用 |
|------|------|------|
| `bt_navigator` | `enable_nav_monitor_echo: true` | goal + behavior_tree 回显 |
| `controller_server` | `enable_nav_monitor_echo: true` | 全局路径 + 阶段切换回显 |
| `nav2_nav_monitor` | `enable: true` | 监控节点总开关 |

`nav2_nav_monitor` 其余参数：

| 参数 | 默认 | 说明 |
|------|------|------|
| `output_dir` | `~/.dexe/navigation/nav_monitor` | JSON 保存目录 |
| `max_retained_files` | `100` | 按 mtime 只保留最近 N 个 |
| `passing_path_sample_period` | `0.5` | 实走轨迹采样周期 (s) |
| `cmd_vel_topic` | `cmd_vel_nav` | 控制器输出速度话题（平滑前） |
| `smoothed_vel_topic` | `control/cmd_vel_nav` | 平滑后速度话题 |
| `desired_vel_topic` | `/nav_monitor/desired_velocity` | 规划期望速度回显话题 |
| `controller_target_topic` | `/nav_monitor/controller_target` | 控制器实际选择的轨迹速度/加速度目标 |
| `odom_topic` | `odom` | 实测速度话题（w1 配置用 `feedback/odom`） |
| `controller_phase_topic` | `/nav_monitor/controller_phase` | 阶段回显话题 |

## Scripts（均可 ros2 run）

```bash
# 批量分析：一条指令 → analysis_<时间戳>/ 子目录
#   summary_tasks.png/csv   按任务横铺 + 总平均（三维误差/阶段均时/速度）
#   summary_points.png/csv  按导航点聚类合并（每点位误差/速度/阶段时间/成功率）
#   tasks/                  每任务的路径图 + 速度图
ros2 run nav2_nav_monitor analyze_nav_tasks.py [目录] [--no-per-task]

# 单任务路径图：地图视图（规划×N + 实走轨迹时间渐变 + 误差标注）+ 偏差曲线
ros2 run nav2_nav_monitor plot_nav_task_path.py <json或目录>

# 单任务速度图：V-T / W-T / A-T / V-S / W-S（规划状态、控制目标、输出、平滑后、实测）
ros2 run nav2_nav_monitor plot_nav_task_speed.py <json或目录>

# 测试数据生成（演示/回归用，独立目录不污染真实记录）
ros2 run nav2_nav_monitor gen_test_nav_tasks.py [--count 10] [--seed 7]
```

传目录时自动取最新一个 `nav_task_*.json`。

**目录缺省解析顺序**（脚本默认跟随节点实际配置的 `output_dir`，改 YAML 无需改脚本参数）：

1. 命令行显式传入的目录/文件
2. 运行中 `nav2_nav_monitor` 节点的 `output_dir` 参数（`ros2 param get`）
3. 节点上次启动写下的 marker：`~/.dexe/navigation/.nav_monitor_last_output_dir`
4. 内置默认 `~/.dexe/navigation/nav_monitor`

脚本启动时会打印实际使用的目录及其来源。

## 误差口径

`goal_reached_error` 按 goal 朝向系分解为三维（与 goal checker lon/lat 容差同一语义）：
**纵向 longitudinal**（沿目标朝向，正=过冲）/ **横向 lateral**（垂直方向）/ **角度 yaw**。
批量分析中精度/速度/阶段时间的均值只统计 succeeded 任务（中止/取消任务的
"误差"是中途停止位置，不是到达精度）。

## FAQ

- **没有生成 JSON？** 依次检查三处开关是否都为 true、`output_dir` 可写、
  导航是否真正到达终态（SUCCEEDED/ABORTED/CANCELED 都会落盘）。
- **阶段时间全在"其他"？** `controller_server` 的 `enable_nav_monitor_echo`
  未开，或 staged_control 未启用（无阶段切换事件）。
- **goal 名字是坐标不是站点名？** NavPoint 名字增强未落地（数据源在
  mobile_manager/navigation_adapter，属跨 workspace 依赖，待定）。
