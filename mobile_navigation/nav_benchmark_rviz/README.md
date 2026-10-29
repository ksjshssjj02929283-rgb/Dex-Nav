# nav_benchmark_rviz

导航 Benchmark 测试面板（RViz）。是命令行工具包 `nav_benchmark_tools`（nbt）的图形入口：界面在这里，测试逻辑在机器人上的 `nbt serve` 里，两者只通过 `/nbt/*` 话题通信（协议见 `nav_benchmark_tools/README.md`）。

## 使用

```bash
# 机器人上
ros2 run nav_benchmark_tools nbt serve
# 机器人桌面或开发机上（同一个 ROS_DOMAIN_ID）
rviz2   # Panels → Add New Panel → nav_benchmark_rviz/BenchmarkPanel
```

## 功能（P0）

| 位置 | 功能 | 说明 |
|---|---|---|
| 顶栏 | 连接状态 | 后端版本、最近一次状态时间；3 s 没收到状态标红 |
| 顶栏 | 测前检查 | 定位、时钟、后端、地图、急停 / 防撞条、必要传感器、电量、导航故障、停障区、点位 / 路线、导航入口、磁盘、版本，逐项红 / 黄 / 绿 |
| 顶栏 | 硬停 | 任何时候都能按：发 `/cmd_vel_hard_stop`，同时取消当前任务、停掉任务序列 |
| 状态 | 状态看板 | 定位、导航、障碍、急停 / 防撞条、电量、后端 / 地图、传感器 |
| 会话 | 测试会话 | 选用例 ID 和标签 → 开始：按用例预设录包 + 采环境快照；结束：录包收尾、写会话汇总 |
| 会话 | 打点 | 预设按钮（放障碍、人员进入、按下急停等）和自定义文字；时刻记进录包和事件表 |
| 任务 | 导航任务 | 从地图资产选路线或点位（可多选），重复轮数、间隔、失败即停；暂停 / 继续 / 取消；限速；结果表和成功率、耗时统计 |
| 事件 | 事件流 | 最近 200 条状态变化、打点、命令和任务结果 |

## 约束

- 开始导航任务前必须勾「已清场」；每次任务序列结束后自动取消勾选，下一次重新确认。
- 面板上的耗时都是诊断口径，用户感知耗时以 XWiz 端计时为准。
- 不提供：地图资产编辑（用 `MapAssetManagerPanel`）、建图开始 / 结束、改参数、自动直线制动（`nbt brake run`，只在命令行用，要回车确认）。

## 测试

```bash
colcon build --packages-select nav_benchmark_rviz --parallel-workers 8
colcon test --packages-select nav_benchmark_rviz && colcon test-result --verbose
```

单测离屏运行（`QT_QPA_PLATFORM=offscreen`），不需要显示器和 ROS 节点；`test/data/status_sample.json` 是 `nbt serve` 实际发出的状态，用来核对面板和后端的协议一致。
