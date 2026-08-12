# mobile_bag_recorder — 导航数据录包节点

取代 `record_rosbag.sh`：录制话题与全部策略参数由 yaml 驱动，`ros2 run` 启动，
磁盘占用有上限。

## 能力

| 能力 | 状态 | 说明 |
|------|------|------|
| 常态分段录制 | ✅ 可用 | 按时长切段，每段是一个可独立回放/独立删除的完整 bag |
| 保留策略 | ✅ 可用 | `rolling` 删最旧 / `limited` 录满即停，段数与总容量双上限 |
| 话题延迟上线自动补录 | ✅ 可用 | 清单里尚未发布的话题不影响启动，上线后自动订阅 |
| 触发式事件录制 | ✅ 可用 | 内存环形缓冲，产出覆盖「触发点前 N 秒 + 后 M 秒」的独立 bag |
| 自动触发源 | ✅ 可用 | 导航 ABORTED / **卡住** / 诊断 ERROR / 终点误差 / 恢复行为 |
| 统一触发接口 | ✅ 可用 | topic（模块上报，非阻塞）+ service（带 reason/窗口覆盖，返回 bag 路径）|

## 快速使用

```bash
# 用默认参数启动
ros2 launch mobile_bag_recorder mobile_bag_recorder.launch.py

# 用自定义参数文件
ros2 launch mobile_bag_recorder mobile_bag_recorder.launch.py params_file:=/path/to/params.yaml

# 直接 run（调试用，命令行覆盖参数）
ros2 run mobile_bag_recorder mobile_bag_recorder_node --ros-args \
  -p output_dir:=/tmp/bags -p continuous.segment_duration:=60
```

输出布局：

```
<output_dir>/
├── continuous/
│   ├── bag-20260812-142347/            # 常态段，每段一个独立 bag
│   └── bag-20260812-142352/
└── event/
    ├── event-20260812-145435-manual/           # 手动触发
    └── event-20260812-150245-goal_error_0_55m/ # 自动触发，名字带原因
```

## 服务接口

| 服务 | 类型 | 说明 |
|------|------|------|
| `~/start_continuous` | `std_srvs/srv/Trigger` | 启动常态录制 |
| `~/stop_continuous` | `std_srvs/srv/Trigger` | 停止常态录制并关闭当前段 |
| `~/trigger` | `mobile_bag_recorder/srv/TriggerRecord` | 触发事件录制；可带 reason/source 与窗口覆盖，返回 bag 路径 |

| 话题 | 类型 | 说明 |
|------|------|------|
| `~/trigger_event` | `std_msgs/msg/String`（内容即 reason）| 导航栈内各模块上报异常，非阻塞、零新依赖 |

```bash
ros2 service call /mobile_bag_recorder_node/stop_continuous std_srvs/srv/Trigger

# 带原因与窗口覆盖（pre/post 传 0 则沿用配置）
ros2 service call /mobile_bag_recorder_node/trigger mobile_bag_recorder/srv/TriggerRecord \
  "{reason: 'collision_risk', source: 'collision_monitor', pre_seconds: 10, post_seconds: 5}"
```

## 统一触发接口（给其他模块用）

两个入口，汇聚到同一套录制逻辑，按调用方的约束选：

| 入口 | 适用调用方 | 特点 |
|------|-----------|------|
| **topic** `~/trigger_event`（`std_msgs/String`）| 导航栈内各模块（controller / planner / costmap / 定位 …）| **不阻塞**，控制回路里也能安全调用；只依赖 `std_msgs`，**不必 depend 本包** |
| **service** `~/trigger`（`TriggerRecord.srv`）| 人工、上位机、mission_manager | 有应答、返回 bag 路径、可按场景覆盖取证窗口 |

模块侧接入只要两行：

```cpp
auto pub = create_publisher<std_msgs::msg::String>("/mobile_bag_recorder_node/trigger_event", 10);
// 检测到自己领域的异常时：
std_msgs::msg::String m; m.data = "controller_stuck"; pub->publish(m);   // 不会阻塞控制周期
```

reason 会写进事件 bag 的目录名，因此现场直接从文件名就能看出这包是为什么留的。

> ⚠️ **不要让 `nav2_controller` 这类核心模块 depend 本包**去用那个 srv——控制器依赖录包工具是架构倒置。
> 模块侧一律走 topic；确需 ack 时应把 srv 提升到 `8_interfaces`，而不是反向依赖。

## 新增一个触发源

自动触发源是 `TriggerSource` 的实现，三步接入，调度器与录制器一行不动：

1. 加 `src/trigger_sources/<name>_source.cpp`，实现 `name()` / `isEnabled()` / `setup()`
2. 在 `include/mobile_bag_recorder/trigger_sources.hpp` 声明 `make<Name>Source()`
3. 在 `src/trigger_sources/builtin.cpp` 的列表里加一行，并把源文件加进 `CMakeLists.txt`

约定：判定取**边沿**（异常出现那一刻）。冷却与并发抑制虽然在录制器侧兜底，但那是最后一道防线，不是让触发源偷懒的理由。

## 事件录制（触发式）

平时消息只进内存环形缓冲；触发后把缓冲倒进一个独立 bag，再续录 `post_seconds` 关闭，
于是这个 bag 恰好覆盖**触发点前 `pre_seconds` 秒 + 后 `post_seconds` 秒**。

自动触发源（逐项可关）：

| 触发源 | 话题 | 判据 | 默认 |
|--------|------|------|------|
| 导航失败 | `/navigate_to_pose/_action/status` | `STATUS_ABORTED` | 开 |
| 导航取消 | 同上 | `STATUS_CANCELED` | 关 |
| 规划诊断 | `/navigation/speed_plan_diagnostics` | `level >= ERROR` | 开 |
| 终点误差 | `/navigation_goal_error` | `distance_error > goal_error_threshold_m` | 关（阈值 0）|
| **导航卡住** | `.../status` + `.../feedback` 或 `odom` | EXECUTING **且** N 秒内推进 < 阈值 | **开** |
| 恢复行为 | `/behavior_tree_log` | 恢复类节点进入 RUNNING | 关，且需 nav2_msgs |

判定都是**边沿触发**（异常出现的那一刻），持续异常不会反复刷 bag；再叠加
`cooldown_seconds` 冷却与「录制中不重复触发」两道抑制。

### 手工注入触发（联调用）

```bash
ros2 run mobile_bag_recorder inject_trigger.py diagnostics 2      # 注入 ERROR 诊断
ros2 run mobile_bag_recorder inject_trigger.py goal_error 0.55    # 注入终点误差
ros2 run mobile_bag_recorder inject_trigger.py nav_aborted        # 注入导航 ABORTED
```

**不要用 `ros2 topic pub` 发 DiagnosticArray**：它的 `level` 是 ROS `byte` 字段，CLI 会拒绝
整数并报 `The 'level' field must be of type 'bytes' or 'ByteString' with length 1`，
命令看似执行其实没发出任何消息。上面的脚本用 rclpy 正确构造了该字段。

## 参数

完整清单见 `params/mobile_bag_recorder_params.yaml`。常用项：

| 参数 | 默认 | 说明 |
|------|------|------|
| `output_dir` | `~/.dexe/navigation/bags` | 输出根目录 |
| `continuous.segment_duration` | `300` | 每段时长（秒）|
| `continuous.mode` | `rolling` | `rolling` 删最旧 / `limited` 录满即停 |
| `continuous.max_segments` | `24` | 磁盘上最多保留的段数（含正在写的那段）|
| `continuous.max_total_size_mb` | `20480` | 总容量上限（MB）|
| `continuous.topics` | 见 yaml | 录制话题清单 |

**段数与容量两个上限同时生效**：段大小随场景浮动，只设段数控不住盘。

## 设计要点

**为什么自己切段而不用 rosbag2 的 `max_bagfile_duration`**
后者切出的多个 `.db3` 共享同一份 `metadata.yaml`，删掉其中的旧文件会让 metadata 与
实际文件不符、整包无法回放。自管切段让每段成为独立完整的 bag，可以安全地按段回收。
代价是切段瞬间几十毫秒的空隙。

**为什么保留策略读磁盘而不是内存计数**
节点重启后内存计数归零，磁盘上的旧段就脱管了——`rolling` 会越攒越多，`limited` 会
变成"每重启一次再录 n 段"。判定依据一律是磁盘上实际存在的段目录。

**QoS 适配**
订阅 QoS 由 `rosbag2_transport::Rosbag2QoS::adapt_request_to_offers` 按发布端实际
offer 推导。不要改成固定 QoS：`/map` 是 transient_local、部分雷达话题是 best_effort，
写死会静默收不到数据。

**写盘不阻塞订阅回调**
`writer_cache_size > 0` 时 rosbag2 内部走 cache + consumer 线程，`write()` 只是入队。

**环形缓冲按时间裁剪，不按字节**
rosbag2 自带的 snapshot 缓冲（`CircularMessageCache`）以字节为上限，窗口长度会随数据率
漂移——同样 200 MB 可能是 60 s 也可能是 15 s。取证场景需要确定的时间窗口，所以这里以
消息时间戳裁剪；`max_buffer_mb` 只作防 OOM 兜底，一旦因它丢消息，触发时会告警提示
窗口可能短于设定值。

**导航卡住怎么判**
"不动"本身不是故障，"该动却不动"才是——所以判据是**任务进行中（action EXECUTING）且持续无实质
推进**的与关系。只看速度会把避障等待、原地旋转对准全报成故障；只看进度停滞则在没有任务时持续误报。
推进量优先用 action 的 `distance_remaining`（能识别"在动但一直绕圈"），拿不到时回退 odom 位移。

**可选依赖 nav2_msgs**
「恢复行为」触发源与停滞检测的 feedback 模式需要它。`find_package(nav2_msgs QUIET)` 找不到时，
前者不编入（设 true 会明确告警）、后者自动回退 odom 模式，其余功能不受影响。
注意：本包若用系统 `/opt/ros/humble` 的 nav2_msgs 编译、而运行时 source 的工作区里有 fork 过的
nav2_msgs，需确认 `BehaviorTreeLog` 与 `NavigateToPose` 的定义一致，否则存在消息 ABI 风险。

## 测试

```bash
colcon build --packages-select mobile_bag_recorder --parallel-workers 8
colcon test --packages-select mobile_bag_recorder --parallel-workers 8
colcon test-result --test-result-base build/mobile_bag_recorder --verbose
```
