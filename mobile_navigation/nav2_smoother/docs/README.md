# nav2_smoother 平滑器说明

本文档说明 `mobile_navigation/nav2_smoother` 中路径平滑器的工作原理、碰撞检测关系和常见失败原因。当前包提供两个插件：

- `nav2_smoother::SimpleSmoother`
- `nav2_smoother::SavitzkyGolaySmoother`

## Smoother Server 流程

`smoother_server` 是 Nav2 生命周期节点，对外提供 `smooth_path` action。它从 `smoother_plugins` 参数加载平滑器插件，接收 planner 输出的 `nav_msgs::msg::Path` 后调用指定插件进行平滑，并发布 `plan_smoothed`。

相关参数：

| 参数 | 说明 |
| --- | --- |
| `smoother_plugins` | 启用的平滑器插件列表 |
| `costmap_topic` | action 层碰撞检查使用的 costmap，默认 `global_costmap/costmap_raw` |
| `footprint_topic` | action 层 footprint 碰撞检查使用的机器人足迹 |
| `robot_base_frame` | footprint 变换使用的机器人基坐标系 |
| `transform_tolerance` | footprint TF 查询容差 |

如果 action goal 中 `check_for_collisions` 为 true，插件平滑结束后，`smoother_server` 会遍历平滑后的路径点，并调用 `CostmapTopicCollisionChecker::isCollisionFree()` 做 footprint 级碰撞检查。这个检查独立于具体 smoother 插件。

## SimpleSmoother 原理

`SimpleSmoother` 是一个迭代式路径平滑器。它先调用 `findDirectionalPathSegments()` 把路径按倒车 cusp 或原地旋转点切成多段，只处理长度大于 9 个点的路径段。

每个被处理的路径段会固定首尾点，反复更新中间点的 `x`、`y` 坐标：

```text
new_point = current
          + w_data * (original_point - current)
          + w_smooth * (next_point + prev_point - 2 * current)
```

含义：

- `w_data` 把平滑点拉回原始路径，避免偏离 planner 结果过大。
- `w_smooth` 把平滑点拉向相邻点平均位置，降低折线抖动和尖角。
- `tolerance` 控制收敛阈值。
- `max_its` 控制最大迭代次数。
- `do_refinement` 开启后会额外递归细化，最多 4 轮。

平滑完成后会调用 `updateApproximatePathOrientations()`，根据相邻路径点方向重新估算姿态。倒车段会翻转 yaw，使路径方向与倒车运动一致。

### SimpleSmoother 的障碍物检测

`SimpleSmoother` 插件内部会做点位级 costmap 检查。每次移动中间路径点后，它会：

1. 用 `worldToMap(x, y, mx, my)` 把世界坐标转换为 costmap 栅格坐标。
2. 用 `getCost(mx, my)` 读取该栅格代价值。
3. 当 `cost > MAX_NON_OBSTACLE` 且 `cost != NO_INFORMATION` 时，认为这次平滑产生不可行碰撞，回退到上一轮路径并返回失败。

Nav2 costmap 关键代价值：

| 常量 | 值 | 含义 | SimpleSmoother 处理 |
| --- | ---: | --- | --- |
| `FREE_SPACE` | 0 | 空闲区域 | 允许 |
| 普通膨胀代价 | 1..252 | 膨胀层梯度代价 | 允许 |
| `INSCRIBED_INFLATED_OBSTACLE` | 253 | 内切膨胀障碍 | 判定失败 |
| `LETHAL_OBSTACLE` | 254 | 致命障碍 | 判定失败 |
| `NO_INFORMATION` | 255 | 未知区域 | 插件内部允许 |

因此，`SimpleSmoother` 的障碍物检测和膨胀层相关，但不是检查整条 footprint，也不是检查所有膨胀代价。它只检查路径点所在栅格是否进入 `253/254`。如果膨胀层把障碍物附近区域标成 `INSCRIBED_INFLATED_OBSTACLE(253)`，平滑点进入该区域就会失败；如果只是普通膨胀梯度 `1..252`，插件内部不会因此失败。

注意：`SimpleSmoother` 当前没有检查 `worldToMap()` 的返回值。如果平滑点越出 costmap 边界，`mx/my` 可能不是有效栅格坐标，这一点排查异常失败或异常行为时需要注意。

## SavitzkyGolaySmoother 原理

`SavitzkyGolaySmoother` 是快速滤波式平滑器，不做迭代优化。它同样会先按倒车 cusp 或原地旋转点切分路径段，只处理长度大于 9 个点的段。

核心是 7 点 Savitzky-Golay 滤波器：

```text
[-2/21, 3/21, 6/21, 7/21, 6/21, 3/21, -2/21]
```

对每个中间点，平滑器取当前点前后最多 3 个邻居，分别对 `x` 和 `y` 坐标做加权求和。中间点权重大，近邻点权重次之，最远两点是轻微负权重，用于保留局部形状并抑制噪声。

边界处理：

- 首点和末点固定。
- 靠近首尾的点通过重复端点补足 7 点窗口，避免越界。

参数：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `do_refinement` | `true` | 是否额外重复滤波 |
| `refinement_num` | `2` | 额外滤波次数 |

`SavitzkyGolaySmoother` 插件内部不保存 `costmap_sub`，也不调用 `worldToMap()` 或 `getCost()`，因此它本身不会因为路径点进入 `253/254` 栅格而返回失败。但如果 action goal 开启 `check_for_collisions`，`smoother_server` 仍会在插件成功后做 footprint 级碰撞检查，最终 action 仍可能失败。

## 规划成功但平滑失败的常见原因

使用 `SimpleSmoother` 时，规划成功但平滑失败通常来自以下几类问题：

- 规划路径贴近障碍物或窄通道，迭代平滑切角后，中间点进入 `INSCRIBED_INFLATED_OBSTACLE(253)` 或 `LETHAL_OBSTACLE(254)`。
- planner 使用的 costmap 与 `smoother_server.costmap_topic` 订阅的 costmap 不一致，例如 planner 基于 global costmap，smoother 却检查 local costmap。
- inflation 半径、机器人 footprint 或 costmap 分辨率导致障碍物附近可通行空间很窄，平滑后的点比原始路径更容易落入内切膨胀区域。
- action goal 的 `max_smoothing_duration` 太短，平滑器超时返回 `was_completed=false`。
- `max_its` 太小或 `tolerance` 太严，迭代无法在限制内收敛。
- 插件内部通过后，action 层 `check_for_collisions=true` 又用 footprint 检查发现碰撞。

排查建议：

1. 确认 `smoother_server.costmap_topic` 是否与期望的 planner costmap 一致。
2. 打开 debug 日志，区分失败来自 `SimpleSmoother` 插件内部还是 `smoother_server` action 层 footprint 检查。
3. 在 RViz 中同时显示原始路径、`plan_smoothed`、costmap 和 inflation 区域，观察平滑点是否切入障碍边界。
4. 对窄通道场景，优先减弱平滑强度，例如降低 `w_smooth`、关闭 `do_refinement`，或增大原始规划路径离障碍物的距离。
5. 若需要快速去噪且希望避免插件内部 costmap 点检查，可评估 `SavitzkyGolaySmoother`，但仍需保留 action 层碰撞检查来保证结果可行。
