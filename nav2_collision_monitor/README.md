# Nav2 Collision Monitor

The Collision Monitor is a node providing an additional level of robot safety.
It performs several collision avoidance related tasks using incoming data from the sensors, bypassing the costmap and trajectory planners, to monitor for and prevent potential collisions at the emergency-stop level.

This is analogous to safety sensor and hardware features; take in laser scans from a real-time certified safety scanner, detect if there is to be an imminent collision in a configurable bounding box, and either emergency-stop the certified robot controller or slow the robot to avoid such collision.
However, this node is done at the CPU level with any form of sensor.
As such, this does not provide hard real-time safety certifications, but uses the same types of techniques with the same types of data for users that do not have safety-rated laser sensors, safety-rated controllers, or wish to use any type of data input (e.g. pointclouds from depth or stereo or range sensors).

This is a useful and integral part of large heavy industrial robots, or robots moving with high velocities, around people or other dynamic agents (e.g. other robots) as a safety mechanism for high-response emergency stopping.
The costmaps / trajectory planners will handle most situations, but this is to handle obstacles that virtually appear out of no where (from the robot's perspective) or approach the robot at such high speed it needs to immediately stop to prevent collision.

![polygons.png](doc/polygons.png)

## Features

The Collision Monitor uses polygons relative the robot's base frame origin to define "zones".
Data that fall into these zones trigger an operation depending on the model being used.
A given instance of the Collision Monitor can have many zones with different models at the same time.
When multiple zones trigger at once, the most aggressive one is used (e.g. stop > slow 50% > slow 10%).

The following models of safety behaviors are employed by Collision Monitor:

* **Stop model**: Define a zone and a point threshold. If more that `N` obstacle points appear inside this area, stop the robot until the obstacles will disappear.
* **Slowdown model**: Define a zone around the robot and slow the maximum speed for a `%S` percent, if more than `N` points will appear inside the area.
* **Approach model**: Using the current robot speed, estimate the time to collision to sensor data. If the time is less than `M` seconds (0.5, 2, 5, etc...), the robot will slow such that it is now at least `M` seconds to collision. The effect here would be to keep the robot always `M` seconds from any collision.

The zones around the robot can take the following shapes:

* Arbitrary user-defined polygon relative to the robot base frame.
* Circle: is made for the best performance and could be used in the cases where the zone or robot could be approximated by round shape.
* Robot footprint polygon, which is used in the approach behavior model only. Will use the footprint topic to allow it to be dynamically adjusted over time.

The data may be obtained from different data sources:

* Laser scanners (`sensor_msgs::msg::LaserScan` messages)
* PointClouds (`sensor_msgs::msg::PointCloud2` messages)
* IR/Sonars (`sensor_msgs::msg::Range` messages)

## Design

The Collision Monitor is designed to operate below Nav2 as an independent safety node.
This acts as a filter on the `cmd_vel` topic coming out of the Controller Server. If no such zone is triggered, then the Controller's `cmd_vel` is used. Else, it is scaled or set to stop as appropriate.

The following diagram is showing the high-level design of Collision Monitor module. All shapes (Polygons and Circles) are derived from base `Polygon` class, so without loss of generality we can call them as polygons. Subscribed footprint is also having the same properties as other polygons, but it is being obtained a footprint topic for the Approach Model.
![HDL.png](doc/HLD.png)

## Configuration

Detailed configuration parameters, their description and how to setup a Collision Monitor could be found at its [Configuration Guide](https://navigation.ros.org/configuration/packages/configuring-collision-monitor.html) and [Using Collision Monitor tutorial](https://navigation.ros.org/tutorials/docs/using_collision_monitor.html) pages.


## Metrics

Designed to be used in wide variety of robots (incl. moving fast) and have a high level of reliability, Collision Monitor node should operate at fast rates.
Typical one frame processing time is ~4-5ms for laser scanner (with 360 points) and ~4-20ms for PointClouds (having 24K points).
The table below represents the details of operating times for different behavior models and shapes:

| | Stop/Slowdown model, Polygon area | Stop/Slowdown model, Circle area | Approach model, Polygon footprint | Approach model, Circle footprint |
|-|-----------------------------------|----------------------------------|-----------------------------------|----------------------------------|
| LaserScan (360 points) processing time, ms  | 4.45 | 4.45 | 4.93  | 4.86  |
| PointCloud (24K points) processing time, ms | 4.94 | 4.06 | 20.67 | 10.87 |

The following notes could be made:

 * Due to sheer speed, circle shapes are preferred for the approach behavior models if you can approximately model your robot as circular.
 * More points mean lower performance. Pointclouds could be culled or filtered before the Collision Monitor to improve performance.

---

## 本仓库相对上游 Nav2 的改动 (Local divergence from upstream)

本包不是上游 Nav2 的纯净拷贝，以下为本项目的本地增改。修改本包或从上游同步时请先看这一节。

### 1. `obstacle_status` 话题（新增）

在 `cmd_vel` 门控之外，额外发布 `nav_interfaces/ObstacleMonitorStatus`（FREE / SLOWDOWN / STOP
三态 + `slowdown_ratio` + `zone_name`），供 `path_follower` 做速度跟踪状态机使用
（见 `path_follower` 的 `VelocityTrackMode`）。相关参数：`obstacle_status_topic`。

上游的 `APPROACH` 动作在这里并入 `SLOWDOWN` 上报——两者对下游都是"降速不停止"的语义。

### 2. `obstacle_status` 空闲保活定时器（新增）

**上游行为**：`process()` 只由 `cmdVelInCallback` 驱动，因此 `obstacle_status` 只在有
`cmd_vel_in` 流入时才发布。

**问题**：这让一个安全信号的发布节奏依赖于下游消费者的输出（依赖倒置），带来两个后果：

1. 下游无法区分"监控节点挂了"和"监控节点没有可反应的输入"——因此无法实现状态超时看门狗。
   `path_follower` 的 `obstacle_status_timeout` 看门狗正是依赖本改动才有意义。
2. 车还没开始动就已存在的障碍，要等第一帧非零速度发布出去才会被上报，此时往往已经晚了。

**改动**：新增 `obstacle_status_idle_rate`（默认 2.0 Hz，`<=0` 关闭）驱动的保活定时器
`idleKeepaliveCallback()`，仅在"最近一个定时周期内没有 `cmd_vel_in` 流入"时才触发一次评估。

设计约束（同步上游时必须保持）：

- **不改动 `cmd_vel` 主通路**。运动中定时器直接跳过，`cmdVelInCallback` → `process()` 的
  行为与延迟特性与上游完全一致。
- **保活路径不发布 `cmd_vel`**：`process()` 新增 `publish_cmd_vel` 参数，保活时传 `false`。
  监控器只应门控速度，绝不能自己产生速度。
- **保活路径只继承 `robot_action_prev_.action_type`，不写 `req_vel`**。`req_vel` 属于
  `cmd_vel` 通路的状态，若被保活路径覆盖成零，`publishVelocity()` 会漏掉下一次真实指令的
  "机器人刚刚停下"边沿，从而不转发那一帧停车指令。
- 保活用零速评估：STOP/SLOWDOWN 多边形的判定是纯几何的（点是否落在多边形内），零速下结果
  完全正确；`APPROACH` 会退化为不触发，而这正是它在零速下应有的语义（无运动即无预期碰撞）。
