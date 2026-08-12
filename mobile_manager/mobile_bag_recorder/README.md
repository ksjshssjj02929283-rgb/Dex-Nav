# mobile_bag_recorder — 导航数据录包节点

取代 `record_rosbag.sh`：录制话题与全部策略参数由 yaml 驱动，`ros2 run` 启动，
磁盘占用有上限。

## 能力

| 能力 | 状态 | 说明 |
|------|------|------|
| 常态分段录制 | ✅ 可用 | 按时长切段，每段是一个可独立回放/独立删除的完整 bag |
| 保留策略 | ✅ 可用 | `rolling` 删最旧 / `limited` 录满即停，段数与总容量双上限 |
| 话题延迟上线自动补录 | ✅ 可用 | 清单里尚未发布的话题不影响启动，上线后自动订阅 |
| 触发式事件录制 | 🚧 开发中 | 内存环形缓冲取触发点前后一段时间 |

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
└── continuous/
    ├── bag-20260812-142347/     # 每段一个独立 bag（含自己的 metadata.yaml）
    └── bag-20260812-142352/
```

## 服务接口

| 服务 | 类型 | 说明 |
|------|------|------|
| `~/start_continuous` | `std_srvs/srv/Trigger` | 启动常态录制 |
| `~/stop_continuous` | `std_srvs/srv/Trigger` | 停止常态录制并关闭当前段 |

```bash
ros2 service call /mobile_bag_recorder_node/stop_continuous std_srvs/srv/Trigger
```

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

## 测试

```bash
colcon build --packages-select mobile_bag_recorder --parallel-workers 8
colcon test --packages-select mobile_bag_recorder --parallel-workers 8
colcon test-result --test-result-base build/mobile_bag_recorder --verbose
```
