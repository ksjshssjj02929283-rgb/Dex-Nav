# Last Session

## 日期
- 2026-09-12

## 本次目标
- `mobile_bag_recorder` 在导航失败触发录包时，把**系统里最近几分钟的日志**一并存进事件 bag 目录。

## 结论
- 日志源选 **systemd journal**，不是 `~/.ros/log/`：
  机器人上所有节点由 `auto_mode.sh` 经 `systemd-cat -t dexe_xxx` 拉起，stdout/stderr 全进 journal，
  那里才是「全系统一条时间线」（还含 kernel / systemd / 网络这些 ROS 侧看不到的事件）；
  而 `~/.ros/log/` 按每次 launch 一个目录组织（pc1 实测已 6500+ 个），切不出时间窗口。
- `[实测 2026-09-12 pc1]` `dexforce` **不用 sudo** 就能读全量 journal（在 `adm` 组），
  `journalctl --no-pager -o short-precise --since "<本地时间>"` 退出码 0；5 min 日志 300~540 KB，
  相对 bag 体积可忽略，所以日志窗口（默认 300 s）开得比数据窗口（`event.pre_seconds` 60 s）更长。
- 采集时机放在**事件 bag 关闭之后**：窗口右端自然盖住 post 段，日志与数据覆盖同一段时间。
- 采集走 fork + exec 子进程 + 后台线程，不占执行器线程（journalctl 扫几分钟日志是几百毫秒量级，
  占着执行器会连带卡住订阅回调——参见 path_follower 单线程执行器被同步等待饿死的旧坑）。

## 本次已修改文件
- `mobile_manager/mobile_bag_recorder/include/mobile_bag_recorder/log_collector.hpp`（新增）
- `mobile_manager/mobile_bag_recorder/src/log_collector.cpp`（新增）
- `mobile_manager/mobile_bag_recorder/test/test_log_collector.cpp`（新增）
- `mobile_manager/mobile_bag_recorder/include/mobile_bag_recorder/recorder_config.hpp`（加 `LogCaptureConfig`）
- `mobile_manager/mobile_bag_recorder/src/recorder_config.cpp`（声明 `log_capture.*` 参数）
- `mobile_manager/mobile_bag_recorder/include/mobile_bag_recorder/event_recorder.hpp` / `src/event_recorder.cpp`
- `mobile_manager/mobile_bag_recorder/CMakeLists.txt`、`params/mobile_bag_recorder_params.yaml`、`README.md`

## 产出布局

```
<output_dir>/event/event-20260912-150012-schedule_seg0_failed/
├── metadata.yaml
├── event-...-_0.mcap
└── logs/system-journal.log      # 文件头：bag 路径 / 触发原因 / 时间窗口 / 采集命令
```

## 参数
| 参数 | 默认 | 说明 |
|------|------|------|
| `log_capture.enabled` | `true` | 关掉即完全不采日志 |
| `log_capture.pre_seconds` | `300` | 触发点之前取多久 |
| `log_capture.max_lines` | `0`（不限）| 某节点刷屏时兜底 |
| `log_capture.timeout_seconds` | `30` | 超时 SIGKILL，已写入部分保留 |
| `log_capture.identifiers` | 空=全量 | 只收指定 `systemd-cat -t` 标识；**不要写 `[]`** |

## 验证
- `colcon build --packages-select mobile_bag_recorder` 通过；`colcon test` 22 tests / 0 failures（新增 4 条）。
- 本机端到端：`~/trigger` 手动触发 → 事件 bag 下生成 `logs/system-journal.log`，58 KB，耗时约 200 ms；
  identifiers 过滤、`max_lines`、无匹配日志（`-- No entries --`）三条路径都验过。
- pc1 上只验了 journalctl 命令本身可用（只读），**新代码尚未部署到机器人**。

## 另一件事：急停不再漏掉路网任务（command_board）

**问题**：`command_board` 急停响应只 `async_cancel_all_goals()` 了 `/navigate_to_pose`
（`command_board_ros.hpp` 里客户端类型写死 NavigateToPose），而路网任务跑在
`NavigateThroughGraphPoints` 上，砍不到。后果不是"任务没结束"这么简单：

- `[实测 2026-09-12 pc1]` 当前部署 `/navigate_to_pose` 的 **Action servers: 0**（bt_navigator 没起），
  那一刀实际打在空气上；路网任务的 server 是 `/schedule_planner_node`，client 是 `/chassis_navigation_real`。
- 急停后 `StopLocalization()` 断掉 map→base_link，path_follower 的 `getCurrentState()`
  拿不到位姿就只发 0 速、**不报 FAILED**（`path_follower.cpp:2240-2264`），
  schedule_planner 于是一直等到 `segment_timeout`（300 s）才超时收尾。
- 更要命的是这期间松开急停、定位恢复，path_follower 的 `route_` 还在，**会自己接着原路径继续走**。

**改法**：急停/碰撞边沿（以及重置地图原点流程的"停导航"那一步）额外调
`/schedule_planner/force_cancel`（`std_srvs/Trigger`）。选它而不是 action cancel 的理由：
无论有没有 active goal 都会兜底停车，且 command_board 不必 depend `nav_interfaces`。
schedule_planner 侧收到后置 `force_cancel_requested_` → 等待循环返回 2 →
goal 以 **ABORTED** 收尾（EXECUTING 不能直接转 CANCELED），并 `~/stop` 掉 path_follower，
所以松开急停不会再自己续跑。cancel 路径不触发 `triggerBagRecord`，不会刷事件 bag。

新参数（代码默认，yaml 里有注释说明）：`cancel_schedule_planner: true`、
`schedule_planner_cancel_service: "/schedule_planner/force_cancel"`。

验证：本机起 command_board + 假 force_cancel 服务 + 假 `/diagnostics_agg`，
按下急停调用一次、按住不重复、松开再按又调一次；服务不存在时打 WARN 且不阻塞。

**注意**：上层 `/chassis_navigation_real` 会看到 goal 是 ABORTED 而不是 CANCELED，
若它把 ABORTED 当导航失败去重试/报警，需要那边一起对齐。

## 第三件事：双 Lifecycle Server 框架适配（①~⑤ 全部落地）

按 `nav2_schedule_planner/Path Navigation双Lifecycle Server框架适配方案.md` 实施，五步都做完并本机验证。

### ① 路径导航栈与 Nav2 栈的资源隔离（前置条件）
`[实测]` 两套栈原本都用节点名 `velocity_smoother`、都吃 `/cmd_vel_nav`、都输出 `/control/cmd_vel_nav`。
同名节点在 DDS 层就打架（两个 `change_state` 服务端，管理器请求随机路由）。改法：
- 节点名 → `velocity_smoother_path` / `collision_monitor_path`（launch 新增 `node_name` 参数，
  管理器名随之 `lifecycle_manager_<node_name>`）
- 栈内速度链 → `/cmd_vel_nav_path`，监控侧输出 → `/cmd_vel_nav_path_sink`
- **输出 `/control/cmd_vel_nav` 刻意不改**：那是 cmd_vel_mux 的既定入口，错开就要改 mux 且多出仲裁语义
- 两个 yaml 顶层 key 改 `/**` 通配（写死节点名的话一改名参数全部静默失效）
- 坑：`{"node_names": [node_name]}` 会被 launch_ros 当成**拼接后的字符串**，必须写 `[[node_name]]`

### ② path_follower 生命周期化
基类换 `nav2_util::LifecycleNode`；构造函数只声明参数；`on_configure` 建接口（timer 不启动）；
`on_activate` 激活 publisher + 起 timer；`on_deactivate` 顺序固定：停 timer → 清路线 →
发零速和 IDLE → **最后**才失活 publisher（反了零速会被 LifecyclePublisher 丢掉）。
- `onStopService` 的清理体抽成 `resetRouteState()`，与 `on_deactivate` 共用
- 输入门：inactive 时 `/path`、`/tracking_path` 直接丢弃并告警，不缓存
- 参数回调改频率时只在 active 才重建 timer（否则 inactive 期间改参数能把控制环复活）

### ③ nav2_schedule_planner 生命周期化
同上；`handleGoal` 加 active 门（rclcpp_action 的 server 没有失活概念，只能在这里拒）；
`on_deactivate` 复用 force_cancel 那条路径 + join 执行线程，超时（`deactivate_timeout`，默认 5s）
返回 FAILURE 而不是硬 cleanup。

### ④ `path_navigation_bringup.launch.py`（新增，路径导航推荐入口）
起 path_follower 栈 + schedule_planner + `lifecycle_manager_path_navigation`，
`node_names: [path_follower_node, schedule_planner_node]`（正序 activate、逆序 deactivate）。
**`bond_timeout` 默认调到 10.0**：path_follower 装载路线时预摆舵角有 3 秒同步等待且独占单线程
执行器，bond 心跳也在这个执行器上，留 nav2 默认的 4.0 会被误判为节点卡死、把整组 deactivate。

### ⑤ command_board 后端选择
新参数 `navigation_backend`（`nav2` | `path_navigation`，唯一真值源，仅启动期读取）+
`path_navigation.{autostart, lifecycle_manager_service, startup_timeout_sec, require_map_ready}`。
选 path_navigation 时 1Hz 轮询等管理器服务，就绪后调一次 STARTUP；选 nav2 则完全不碰。

### 验证（本机，全部实测）
| 场景 | 结果 |
|---|---|
| 改名后拓扑 | `/cmd_vel_nav_path` 1 发 2 收、`/cmd_vel_nav` 在路径栈里彻底消失、旧节点名消失 |
| AT-01/02 | 两个 Server 初始 unconfigured；管理器 configure(follower)→configure(schedule)→activate(follower)→activate(schedule) |
| AT-03 | inactive 时 goal 被明确拒绝（日志带当前状态）；unconfigured 时 action server 压根不存在 |
| AT-04 | inactive 收到 `/tracking_path` → "Not active (inactive), dropping"，零 cmd_vel |
| AT-05 | 跟踪中 deactivate：schedule 1.48s 收口（goal ABORTED "force-cancel requested"），follower 最后一条 cmd_vel = (0,0,0) + IDLE |
| AT-06 | 再 activate 不继承旧路线（零 cmd_vel） |
| AT-08 | `navigation_backend=nav2` 时两个 Server 保持 unconfigured、无速度输出 |
| ⑤ 编排 | autostart:=false 起栈 → command_board 起来 1.0s 后发 STARTUP，1.2s 内两个 Server 全 active |

### 踩到的坑（记下来别再踩）
- **`path_follower.yaml` 里 `use_sim_time: true`**（production launch 会用 false 覆盖，所以真机没事）。
  但用 `ros2 run --params-file` 单独起时它生效，ROS 时钟停在 0，**所有 `*_THROTTLE` 日志被永久抑制**
  （throttle 判据是 `now - last >= period`，0-0 永远不满足）。排查"回调没执行"时先看这个。
- deactivate 时 rcl 会打一行 `timer is canceled` 的 error-state 报告（执行器已把 timer 收进 waitset
  才被 cancel），以及 bond 的 typesupport 噪声。都是噪声，节点状态正常。

### ⑤ 的补齐：后端选择做成对称的（两套都能被参数管）
`[实测]` 原先 `navigation_backend` 只管得住路径导航那套——`w1_nav2_actual.launch.py` 里
`autostart` 写死 `'True'`，Nav2 自带的 `lifecycle_manager_navigation` 一起来就把整套拉到 active，
跟参数无关。于是两条 launch 同时起 = 两套同时 active。补了两处：
- `w1_nav2_actual.launch.py`：`autostart` 变成 launch 参数（默认仍 True，不改变既有行为）
- command_board：启动状态机按 `navigation_backend` 收敛到「被选中后端的管理器」，
  新增 `nav2.{autostart, lifecycle_manager_service, startup_timeout_sec}`；
  `nav2.autostart` 默认 **false**（Nav2 那套目前自激活，再发一次 STARTUP 是多余噪声）

**三处必须配对**（少一处就会两套同时出速度、都落到 cmd_vel_mux 的 `control/cmd_vel_nav`）：
`w1_nav2_actual.launch.py autostart:=false` + `path_navigation_bringup.launch.py autostart:=false`
+ yaml 里 `navigation_backend` 及对应后端的 `autostart: true`。

验证：`navigation_backend=nav2` + `nav2.autostart:=true` → command_board 向
`/lifecycle_manager_navigation/manage_nodes` 发出 `command=0`(STARTUP)，同时路径导航那套
保持 unconfigured。

### auto_mode.sh（在 dexe_mobile_application 仓）
路径导航两条 launch 合并成一条 `path_navigation_bringup.launch.py autostart:=true`
（journal 标识 `dexe_schedule_planner`/`dexe_path_follower` → 合并为 `dexe_path_nav`）；
Nav2 那行保持注释，旁边写清了"两套并存"要改的三处。
`[实测]` **机器人真正跑的是 `/home/dexforce/xumingkang/auto_mode.sh`**（`dexe-auto.service` 拉起），
不是 `~/w1/dexe_mobile_application/` 下那份；仓库这份改了不等于机器人改了。

### 已切到"两套常驻 + 参数选后端"模式（用户要把新 command_board 推上车）
- `auto_mode.sh`：Nav2 那行解开注释并加 `autostart:=false`；路径导航那行也改 `autostart:=false`
- `command_board_params.yaml`：`navigation_backend: "path_navigation"`，两套的 `autostart` 都留 true
  （**切后端只改 navigation_backend 一行**）
- 验证：用真实参数文件起 command_board → 路径导航栈 1.2s 内 active，假的
  `/lifecycle_manager_navigation/manage_nodes` **一次都没被调**；反过来选 nav2 也验过
- 修了自己引入的一个语法错误：`w1_nav2_actual.launch.py` 的 `description=(` 少一个右括号，
  launch 直接加载失败（`py_compile` + 实起验证）

### 资源实测：未激活的那套花多少
`[实测]` 桌面机 30s 采样，`autostart:=false` 的 Nav2 栈（7 个进程全 unconfigured）：
每进程 0.03~0.10% 单核，**合计 ≈0.43% 单核**，RSS 26~31 MB/进程（**合计 ≈200 MB**）。
未激活的 path_follower/schedule_planner：0.00% / 0.06%。按 velocity_smoother 在两机上的
同工况比值（桌面 1.23% vs 车上 1.6%）折算，车上 ≈0.6% 单核。

`[实测 2026-09-12 pc1]` 车上当前（路径导航 active，29415s 累计）：path_follower 5.1%、
collision_monitor 4.8%、velocity_smoother 1.6%、schedule_planner 0.23%（均按单核）。
内存 15.7G 总 / 5.0G available。**core 4 实测 ~85% 占用**——auto_mode.sh 把 lidar/imu/calib/
localization/chassis/nav 全 `taskset -c 3-4`，那两个核本来就是热点。

**结论**：常驻未激活的那套 CPU 可忽略（<1% 单核），真正成本是 ~200MB 内存和 +7 个进程。
但有一个 6.4% 的浪费点：`support_autostart` 默认 true，选 nav2 时路径导航的
collision_monitor_path + velocity_smoother_path 仍在跑（车上 4.8%+1.6%），而 Nav2 那套
自己不带 collision_monitor——选 nav2 时应给路径导航那行加 `support_autostart:=false`。

### 运行时切后端：`~/set_navigation_backend` 服务（**本版本未开放**：srv 未提交、服务已从 command_board 移除，下面是历史记录；`SwitchNavigationBackend` 函数保留，srv 备份在 dexe_interfaces 本地分支 `backup/xmk051-backend-srv`）
接口 **`nav_interfaces/srv/SetNavigationBackend`**（按要求放在 submodule，不在 command_board）：
`string backend` + `bool force` → `bool success, string message, string active_backend`。

流程：取消导航（`/navigate_to_pose` cancel_all + schedule_planner force_cancel）→ PAUSE 旧后端
→ STARTUP 新后端 → 失败则回滚 STARTUP 旧的。另发 `~/navigation_backend`（latched String）
给 UI 看当前后端。`navigation_backend` 参数降级为**开机默认值**，运行中 `param set` 会被
参数回调**明确拒绝**并提示用服务（不拦的话现场会以为切了、其实速度源没换）。

实现要点：切换服务与两个管理器客户端放在同一个 **Reentrant** 回调组——服务回调里同步等
管理器应答，同组必须能并行处理，否则自锁（command_board 本来就是 MultiThreadedExecutor）。
并发切换用 atomic 挡住，一次只允许一个。

`[实测]` 切 nav2 **1.65s** 完成：path 栈 active→inactive，假的 nav2 管理器收到 `command=0`；
切回来两个 Server 恢复 active；重复切返回 "already on"；非法后端名拒绝；`ros2 param set` 被拒。

**踩坑**：第一次测失败是因为上一轮 launch 没杀干净，同一 domain 上跑了两套同名节点，
`change_state` 请求被随机路由 → 管理器 1ms 内报 "Failed to change state"、重试又撞上
"transition not registered"。**这正是①要隔离节点名的那个故障模式**，误打误撞复现了一次。

### 启动顺序问题（command_board 比导航栈早起）
`[实测]` 不影响。三层兜底：① 1Hz 轮询等管理器服务，预算 `startup_timeout_sec`（默认 60s）；
② 管理器侧 `nav2_util::LifecycleServiceClient` 构造函数死等节点的 `get_state` 服务，
所以 STARTUP 发早了只会阻塞、不会误判失败；③ STARTUP 返回失败时**在预算内每秒重试**（本次新加，
原来是一次失败就收摊）。实测：command_board 先起、20s 后才起导航栈 → 第 27.0s 发 STARTUP，
0.24s 拿到 ack，两个 Server active。

同时修掉一个坑：启动 STARTUP 失败/超时后，再调 `set_navigation_backend` 同一后端会被
"already on" 挡回去。现在加了 `backend_active_`，只有**确实 active** 才短路，否则真去重试。

### demo/单独调试用的 launch 修回可用
生命周期化之后 `path_follower_node` 起来停在 unconfigured，**5 个单独启动的 launch 全部失效**
（节点在、但不收路线不出速度）。给它们各自补了一个只管 path_follower_node 的
`lifecycle_manager_path_follower`（autostart=true，**bond_timeout=10s** 同样是为了躲预摆舵角
那 3 秒同步等待）：
`path_follower.launch.py`、`path_follower_with_collision.launch.py`、`bezier_demo.launch.py`、
`corner_bezier_demo.launch.py`、`tracking_path_demo.launch.py`。
`package.xml` 补 `<exec_depend>nav2_lifecycle_manager</exec_depend>`。

`path_follower_with_collision_and_smoother.launch.py` **刻意不加**：它是生产链路的一环，
path_follower 由 `lifecycle_manager_path_navigation` 统一编排，再塞一个管理器就是两个管理器
抢同一个节点。

`[实测]` `tracking_path_demo` / `bezier_demo` 单跑：path_follower 与 collision_monitor_path
都能到 active。注意 `tracking_path_demo` 不起 collision_monitor，`/obstacle_status` 一直是 stale，
路线会被 "Refusing new path: obstacle status already stale" 挡掉——这是**改动之前就有的**
障碍看门狗行为（`git diff` 里那段代码没动过），跟生命周期无关。

### `w1_nav2_actual.launch.py` 的 autostart 参数化（一度回退，后确认保留）
`autostart` 做成 launch 参数（默认 `'True'`，单独起该 launch 行为不变），透传给
`navigation_launch.py`。`[实测]` `autostart:=false` 时 nav2 六个节点全部停在 unconfigured。

`[实测]` 顺带验到一条：**未声明的 launch 参数是静默忽略的**（`ros2 launch ... nosucharg:=false`
无任何报错/警告）。所以哪天这个 autostart 参数要是被谁改没了，auto_mode.sh 里传的
`autostart:=false` 会被悄悄吃掉、nav2 照样自激活 —— 两套同时出速度，现场极难发现。
这条警告写进了 auto_mode.sh 的注释。

### auto_mode.sh：路径导航写进发布版（用户选的方案）
`[实测]` 该文件已随 `dexe_mobile_application` 切到 `dev` 分支并移位到
**`startup.d/w1_pro/001/pc1/auto_mode.sh`**（老的 `startup/auto_mode.sh` 连同我早先两轮编辑
一起没了；全仓只有这一份带 `w1_nav2_actual`）。发布版与机器人上那份 overlay 脚本的区别：
发布版全程 `systemd-cat`、nav2 那行是**启用**的、**没有** xumingkang overlay 段。

改法（已落地）：nav2 与路径导航两条都用 `autostart:=false`，由 command_board 按
`navigation_backend` 选一套 STARTUP：
```bash
taskset -c 3-4 systemd-cat -t dexe_w1_nav2_actual -p info ros2 launch w1_nav2 \
    w1_nav2_actual.launch.py autostart:=false &
taskset -c 3-4 systemd-cat -t dexe_path_nav -p info ros2 launch nav2_schedule_planner \
    path_navigation_bringup.launch.py autostart:=false &
```
`~/.dex_env.sh` 已经 `source /home/dexforce/w1/install/setup.bash`，所以路径导航进发布后
不需要再单独 source overlay。

**前置条件**：`path_follower` 与 `nav2_schedule_planner` 必须进正式发布——
`[实测]` 当前 `~/w1/install` 里**没有**这两个包（只在 overlay `~/xumingkang/.../install`）。

### 遗留
- 机器人 `auto_mode.sh` 现在分别起 `path_follower_with_collision_and_smoother.launch.py` 和
  `schedule_planner.launch.py`——这两个 launch 现在**不会**把 Server 拉到 active。上机前必须把
  这两条换成一条 `ros2 launch nav2_schedule_planner path_navigation_bringup.launch.py`。
- `mobile_bag_recorder` 在 `feature/linfeng.guo/add_bag_record` 分支上（本分支没有），
  那边的录制话题清单要补 `/cmd_vel_nav_path`（`/cmd_vel_nav` 在路径导航模式下已经没有了）。
- release（`~/w1/install`）里没有 path_follower / nav2_schedule_planner，只有 overlay 有；
  `navigation_backend` 参数要生效，command_board 得随 release 更新。

## 后续
- 部署到 pc1 后跑一次真实导航失败，确认 `logs/` 里能捞到 `dexe_path_follower` /
  `dexe_schedule_planner` 那几路的报错。
- 真机验证急停取消路网任务：拍急停 → 看 schedule_planner 是否立刻 "Goal force-stopped (aborted)"，
  松开后机器人不应自己继续走。注意机上 `/path_follower_node` 和 `/chassis_navigation_real`
  都是两个同名实例，确认请求落在哪个上。
- 若现场发现日志噪声太大（localization 刷屏是已知背景噪声），再考虑用
  `log_capture.identifiers` 收敛，而不是缩短窗口。
