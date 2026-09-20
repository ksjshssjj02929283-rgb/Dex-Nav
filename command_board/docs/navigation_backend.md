# 导航后端选择与运行时切换

整机上有**两套互斥的导航后端**，`command_board` 负责选谁生效：

```text
                       ┌─ nav2 ────────────────────────────────────────────────┐
                       │ w1_nav2_actual.launch.py -> nav2_bringup              │
上层任务 ─────────────►│ /navigate_to_pose (bt_navigator/planner/controller)   │
                       │ 归 lifecycle_manager_navigation 管                    │
                       └───────────────────────────────────────────────────────┘
                       ┌─ path_navigation ─────────────────────────────────────┐
                       │ path_navigation_bringup.launch.py                     │
上层任务 ─────────────►│ /navigate_through_graph_points                        │
                       │   schedule_planner ──/tracking_path──► path_follower  │
                       │ 归 lifecycle_manager_path_navigation 管               │
                       └───────────────────────────────────────────────────────┘
                                        │
            两套的速度最终都进 cmd_vel_mux 的 /control/cmd_vel_nav
            ——所以**任何时刻只能有一套是 active**，这就是本机制存在的理由。
```

两套 Server 都随 `auto_mode.sh` 常驻，但都以 `autostart:=false` 启动，停在
unconfigured：不接任务、不出速度。由 `command_board` 把选中的那套拉起来。

未激活的那套几乎不花钱：实测 7 个 unconfigured 的 nav2 进程合计 **≈0.43% 单核 / ≈200 MB**
（unconfigured 的节点没有订阅、没有 timer、costmap 还没创建）。

---

## 开机默认值：`command_board_params.yaml`

```yaml
navigation_backend: "path_navigation"   # "nav2" | "path_navigation"

path_navigation:
  autostart: true
  lifecycle_manager_service: "/lifecycle_manager_path_navigation/manage_nodes"
  startup_timeout_sec: 60.0
  require_map_ready: false   # true = 等 MapState 有地图名再 STARTUP

nav2:
  autostart: true
  lifecycle_manager_service: "/lifecycle_manager_navigation/manage_nodes"
  startup_timeout_sec: 60.0
```

`autostart` 的含义是「**被 `navigation_backend` 选中时**是否自动 STARTUP」，所以两套都留
`true`，切后端只改 `navigation_backend` 一行。

启动时 `command_board` 以 1 Hz 轮询等对应管理器的服务出现，就绪后发 STARTUP（失败会在
预算内重试）；整个 `startup_timeout_sec` 用完仍没起来，就打一条 ERROR 说明情况并停止重试
（见下一节）。

### 启动顺序：command_board 比导航栈早起，没关系

`auto_mode.sh` 里 `command_board` 排在两个导航后端之前（导航那两条前面还各有一个
`sleep 5`）。这不影响激活，三层机制兜着：

1. `command_board` 启动后以 **1 Hz 轮询**等对应管理器的服务出现，最长等
   `startup_timeout_sec`（默认 60 s），期间只是安静地等。
2. 就算它在管理器服务刚出现的瞬间就发 STARTUP，也不会误判失败——管理器侧的
   `nav2_util::LifecycleServiceClient` 构造函数**会死等**每个托管节点的 `get_state`
   服务，即 STARTUP 会一直阻塞到那套节点齐了才往下走。
3. 万一 STARTUP 返回失败（典型：那套里有节点还没起全），在 60 s 预算内**每秒重试**，
   不是一次失败就收摊。

实测（开发机）：command_board 先起、**20 s 后**才起导航栈 → 它在第 27.0 s 发出 STARTUP，
0.24 s 后拿到 ack，两个 Server 变 active。

超时后放弃只打 ERROR，不会让节点退出；原因修好后随时可以用
`~/set_navigation_backend` 服务把它拉起来（此时不需要 `force:true`——启动没成功过时
不会走"already on"短路）。真机冷启动如果发现 60 s 不够（一堆传感器同时起会拖慢
`ros2 launch`），调大 `startup_timeout_sec` 即可。

> ⚠ **三处必须配对**，少一处就会两套同时出速度（都落到 `/control/cmd_vel_nav`）：
> `w1_nav2_actual.launch.py autostart:=false` ＋
> `path_navigation_bringup.launch.py autostart:=false` ＋
> 本文件里选中后端的 `autostart: true`。
> 某套 launch 若还带着 `autostart:=true`，它自己的 lifecycle_manager 会把自己拉到 active，
> `command_board` 管不着。

---

## 运行时切换：`~/set_navigation_backend` 服务

改 yaml 要重启整个 auto 服务，太重。运行中切换走服务：

```bash
# 切到 Nav2 自主导航
ros2 service call /command_board/set_navigation_backend \
    nav_interfaces/srv/SetNavigationBackend "{backend: 'nav2'}"

# 切回路径导航
ros2 service call /command_board/set_navigation_backend \
    nav_interfaces/srv/SetNavigationBackend "{backend: 'path_navigation'}"

# 卡在中间态时重做一遍（当前已经是该后端也照做）
ros2 service call /command_board/set_navigation_backend \
    nav_interfaces/srv/SetNavigationBackend "{backend: 'nav2', force: true}"
```

| 字段 | 含义 |
|---|---|
| `backend` | `"nav2"` 或 `"path_navigation"`，其它值直接拒绝 |
| `force` | `true` = 当前已经是该后端也重做一遍 |
| `success` | `true` 时**新后端已经 active**（服务是同步的，返回即生效） |
| `message` | 失败原因；成功时是 `switched to 'x'` 或 `already on 'x'` |
| `active_backend` | 本次调用结束时实际生效的后端；失败回滚时即原来那套 |

当前生效的后端同时发布在 **`/command_board/navigation_backend`**（`std_msgs/String`，
transient_local latched），UI 直接订阅即可：

```bash
ros2 topic echo --once /command_board/navigation_backend
```

### 切换时做了什么

```text
1. 取消在跑的导航   /navigate_to_pose cancel_all
                    + /schedule_planner/force_cancel
                    （两刀都发，对不在的那套是无副作用的空调用）
2. PAUSE 旧后端     manage_nodes command=1 → 旧后端 deactivate（保留 configured，切回来快）
3. STARTUP 新后端   manage_nodes command=0
4. 失败回滚         第 3 步失败 → 重新 STARTUP 旧后端，避免两套都停着 = 彻底不能导航
```

先取消再动生命周期是有意的：直接 PAUSE 会让正在执行的任务以「节点被拔掉」的方式死掉，
上层拿不到明确终态。

实测一次切换约 **1.6 s**（路径导航侧 `schedule_planner` 收口约 1.5 s 是主要耗时）。
单次管理器调用的超时是 `backend_switch_timeout_sec`（默认 20 s）；同一时刻只允许一个切换在跑，
并发请求会被拒。

### 不要用 `ros2 param set` 切

```console
$ ros2 param set /command_board navigation_backend nav2
Setting parameter failed: navigation_backend is the boot-time default only; switching at
runtime must go through the ~/set_navigation_backend service (...)
```

`navigation_backend` 只在启动期读一次。参数回调会**明确拒绝**运行时修改——不拦的话现场
会以为已经切了，实际速度源没换，比直接报错危险得多。

**服务切换不回写 yaml**：重启后仍按 yaml 里的值启动。要改开机默认值就改 yaml。

---

## 排查

| 现象 | 多半是 |
|---|---|
| 启动日志 `... never appeared within 60s` | 选中后端那套 launch 没起（或管理器名字配错） |
| `failed to start 'x': ... returned false` | 那套里有节点拒绝转换；去它自己管理器的日志看是哪一个 |
| `rolled back to 'y'` | 新后端起不来，已经回滚，机器人仍能用旧后端 |
| `rollback ... ALSO FAILED` | **两套都不是 active，机器人不能导航**，需要人工介入 |
| `another backend switch is already in progress` | 上一次切换还没结束，等它 |
| 切换成功但机器人不动 | 检查是不是另一套 launch 带着 `autostart:=true` 把自己也激活了，两套在抢 `/control/cmd_vel_nav` |
| `Failed to change state` + `transition not registered` | **同名节点**：同一 DDS 域里跑了两套同名进程，`change_state` 请求被随机路由 |

查当前状态：

```bash
ros2 topic echo --once /command_board/navigation_backend   # command_board 认为谁生效
ros2 lifecycle get /path_follower_node                     # 路径导航侧
ros2 lifecycle get /schedule_planner_node
ros2 lifecycle get /controller_server                      # Nav2 侧
ros2 lifecycle get /bt_navigator
```

两边的状态和话题里的后端名对不上，就说明有人绕过 `command_board` 直接调过
`manage_nodes` 或 `ros2 lifecycle set`。

---

## 相关

- 生命周期编排与两套栈的资源隔离：`mobile_navigation/nav2_schedule_planner/launch/path_navigation_bringup.launch.py`
  与 `path_follower/launch/path_follower_with_collision_and_smoother.launch.py` 的文件头
- 设计背景：`mobile_navigation/nav2_schedule_planner/Path Navigation双Lifecycle Server框架适配方案.md`
- 接口定义：`nav_interfaces/srv/SetNavigationBackend.srv`
