# nav_benchmark_tools

导航 Benchmark 测试工具包。统一入口 `nbt`，按子命令执行不同的测试辅助操作。只订阅话题、调用现有接口，不改导航行为。

```bash
source /home/dexforce/w1/install/setup.bash        # 机器人上；开发机 source 工作区 install
ros2 run nav_benchmark_tools nbt -h                # 子命令列表
ros2 run nav_benchmark_tools nbt <子命令> -h        # 子命令参数
```

## 子命令

| 子命令 | 做什么 | 对应用例 |
|---|---|---|
| `env` | 采主机、软件包版本、导航后端、当前地图、定位状态、各 SOP「测前读参数」里的参数，输出 `env.md` / `env.json` | 每轮开头（通用约定 3.1） |
| `watch` | 盯建图 / 定位 / 导航 / 系统 / 障碍 / 急停状态，关键字段一变就打印一行并写 `events.csv`，状态码带含义 | 全部用例的过程观察 |
| `record --case <ID>` | 按用例录包：通用约定 5.1 默认话题 + 用例族追加话题；录前提示缺失话题；`--split-min` 长跑分段 | 全部用例的数据采集 |
| `nav goto/pose/route` | 经 `/navigation/navigate` 下发点位 / 位姿 / 路线任务，`--repeat` 多轮，打印过程与结果，追加 `~/nav_eval/nav_tasks.csv` | G-FREE、G-ROUTE、G-DOCK、G-STB |
| `nav pause/resume/cancel/speed/state` | 经 `/navigation/nav_command` 暂停、恢复、取消、限速；打印当前 `/navigation/state` | G-SYS-03、G-MOT-02、G-STOP |
| `assets` | 经 `/map_asset_manager/get_assets` 读点位、路线、路网节点与边、区域、房间、失效项 | G-ROUTE、G-FREE-02、S-TRAY 测前核对 |
| `res nav` | 导航进程 CPU / 内存，纯算法、带驱动两个口径，各组运行在哪个核 | G-SYS-04 |
| `res map` | 建图进程 CPU、峰值内存、地图生成耗时 | G-MAP-03 |
| `res stb` | 长跑资源趋势：内存增长率、CPU、进程重启 / 退出 | G-STB-02 |
| `brake run` | 实时触发硬停或平滑器停车，测制动距离 | G-MOT-04 |
| `brake analyze <包>` | 离线分析包里的制动段（硬停、障碍 STOP、急停、取消 / 失败、指令归零） | G-MOT-04、G-STOP |
| `pathnav` / `speed` / `navlog` | 路网（path_navigation）任务执行、速度档、离线分析，见下方「能力摸底速度测试」 | 能力摸底推荐 / 最大速度测试、S-TRAY |
| `serve` | RViz 测试面板的后端，常驻在机器人上：接收面板命令（测前检查、会话录包与打点、导航任务、硬停），发布状态 | 配合 `nav_benchmark_rviz` 面板 |

## 常用流程

```bash
nbt env --label "第二轮 G-ROUTE"                     # 1. 记环境
nbt assets --kinds points,routes,edges               # 2. 核对资产
nbt record --case G-ROUTE-03 &                       # 3. 录包
nbt watch --groups nav,loc,obs &                     # 4. 盯状态
nbt nav route R1 --repeat 5                          # 5. 跑任务
```

`nbt` 是 `ros2 run nav_benchmark_tools nbt` 的简写，需要的话在 `~/.bashrc` 里加 `alias nbt='ros2 run nav_benchmark_tools nbt'`。

## 能力摸底速度测试（path_navigation 路网）

调度规划器（`/navigate_through_graph_points`）的任务不经过 nav2_nav_monitor，这组命令补齐「下发 → 判停稳 → 读导航反馈 → 录划线结果 → 录包 → 离线分析」：

| 子命令 | 做什么 |
|---|---|
| `pathnav plan --start <起点> --key <关键点位,…>` | 从现场地图的路网生成任务计划：导航点直连边上逐段最短路（调度规划器要求相邻点直接有边），每到关键点位切一个任务，自动取最长直线前进边作关键路段，逐任务解析校验；输出 plan.yaml、预览图 plan.svg，`--tutorial` 再生成按现场地图的模拟教程（模板 `resources/tutorial_template.html`）；`--assets` 用 `nbt assets --json` 导出的文件离线生成 |
| `pathnav example` | 打印手写计划的模板（生成不了、需要手工调整时参考） |
| `pathnav targets --batch <批次>` | 读地图资产点位，存 `<批次目录>/targets.json`（目标位姿以它为准） |
| `pathnav resolve <计划>` | 只解析不执行：每个任务能否解析、路线长度、倒车段数 |
| `pathnav run <计划> --batch <批次> --laps N --record` | 按计划下发路网任务；每个任务判停稳（线速度 < 0.01 m/s 且角速度 < 0.02 rad/s 持续 0.5 s），取停稳后 1 s 的 map → 底盘平均位姿对目标点算纵 / 横 / 朝向误差；终点是关键点位时暂停录入划线判定（y 合格 / n 不合格 / x 受干扰），只跑 1 圈时先提示拍照留存；每任务一行写 `tasks.csv`；`--record` 同时录包 |
| `pathnav cancel` | `/schedule_planner/force_cancel`：停车并取消当前任务 |
| `pathnav mode [--watch N]` | 判定当前导航模式：command_board 配置的后端、两套导航节点的 lifecycle 状态、调度规划器 mode → nav2 路网 / path_navigation·path_follower / path_navigation·controller；`--watch` 统计 /cmd_vel_nav_path 与 /cmd_vel_nav 谁在出速度；`pathnav run` 开始前自动检查，非 path_follower 拒绝开始 |
| `speed profile show / set <档位> / restore` | 看、改、恢复跟踪器（及 `--with-smoother` 时平滑器）速度上限；门禁：无进行中任务、车已停稳、平滑器上限够；第一次改档前存基线 |
| `record --case SPEED` | 速度测试录包预设（四级速度、路网任务 action 话题、参考路径、关节） |
| `navlog convert <会话或录包>` | 录包 → nav2_nav_monitor 同格式 `nav_task_*.json`（停稳后误差 + 上报时误差），可直接用 nav_monitor 的 `analyze_nav_tasks.py` 出图 |
| `speed analyze <批次目录>` | 关键点位导航精度与重复精度、记录表 1 / 表 2、关键路段速度与 ±5% 稳态时长、峰值加减速度与 jerk、四级速度限制环节、满载关节偏差 |

批次目录默认 `~/nbt_eval/capability/<批次号>/`：

```text
<批次号>/
  targets.json                  地图资产点位
  profile/                      original.json（第一次改档前）、每次 set / restore 的读回存档
  sessions/<批次号>-<时间>/      每次 pathnav run 一个会话：plan.yaml、run.json、tasks.csv、targets.json、rosbag/
  analysis/                     speed analyze 输出：summary.md、points.csv、stops.csv、tasks_metrics.csv、segment.csv、nav_tasks/
```

运行号 = `批次号-日期-序号`，同一批次跨会话连续编号；中断后用 `--start-lap` / `--start-task` 续跑。

## RViz 面板

图形入口在 `nav_benchmark_rviz` 包（RViz 面板 `nav_benchmark_rviz/BenchmarkPanel`）。面板只管按钮和显示，逻辑在 `nbt serve` 里：

```bash
ros2 run nav_benchmark_tools nbt serve            # 机器人上常驻，结果在 ~/nbt_eval/
```

面板与后端只走 `/nbt/*` 话题，载荷是 JSON 字符串（`version` 字段标协议版本）：

| 话题 | 方向 | 内容 |
|---|---|---|
| `/nbt/cmd` | 面板 → 后端 | `{"version": 1, "id", "op", "args"}`；op 见 `serve_cmd.py` 开头 |
| `/nbt/status` | 后端 → 面板 | 锁存，2 Hz：状态看板、测前检查、会话、导航任务、上一条命令的应答 |
| `/nbt/events` | 后端 → 面板 | 状态字段变化、打点、命令执行、任务结果 |
| `/nbt/marker` | 后端发布 | 打点文字；会话录包时一并录下，离线分析时对齐人工操作的时刻 |

只用话题是有意的：开发机（Fast DDS）和机器人（Cyclone DDS）之间话题能通、服务和动作不通，面板在两边都能用。

## 说明

- 输出默认在 `~/nav_eval/` 下，按子命令和时间建目录；`-o` 指定。
- `nav` 打印的耗时是本机「发出目标 → 收到结果」，属于诊断口径；用户感知耗时以 XWiz 端计时为准。
- `res nav` / `res stb` 的进程归组规则按 `dexe_mobile_application` 的启动脚本（`startup.d/w1_pro/001/pc1/auto_mode.sh`、`run_system.sh`）编写；启动脚本改了进程组织方式，要同步改 `res_nav.py` 的规则，或用 `--rules` 追加。
- `res nav` / `res map` / `res stb` 只读 `/proc`，不需要 ROS 节点在线；其余子命令需要 source 含 `nav_interfaces` 的环境。
- 整机 GPU、温度、PC2 资源用测试侧的 `nx_monitor.py`，与 `res` 子命令同时用时把 `nx_monitor` 的采样率降到 1 Hz。

## 测试

```bash
colcon build --packages-select nav_benchmark_tools --parallel-workers 8
colcon test --packages-select nav_benchmark_tools && colcon test-result --verbose
```
