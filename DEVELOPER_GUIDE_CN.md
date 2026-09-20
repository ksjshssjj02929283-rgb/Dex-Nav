# Dexe Mobile Nav 开发者理解手册

> 面向准备修改、评审和维护本项目的开发者。本文不重复节点拓扑，而是解释设计决策、职责边界、演化成本，以及提交代码前应证明的行为。
>
> 依据：当前工作区源码、参数、launch 和设计文档的静态核查。**实现事实**指当前代码能支持的结论；**架构判断**指根据实现推导的收益与代价；**建议**指尚未实现的改进。没有版本历史或运行证据的地方，不把推断写成开发者原话，也不把配置意图视为部署事实。

## 一、先形成开发者的判断方式

阅读者关注“函数做了什么”；参与开发还需要回答：

1. 这个状态由谁拥有，其他模块通过什么契约使用它？
2. 这次变更会改变哪些消费者的行为？
3. 输入晚到、重复、失效或操作只完成一半时，系统会怎样？
4. 改进一个正常场景，会不会破坏已有的停车、取消、恢复语义？

对这个项目，我的总体判断是：**它优先让现场行为可控、故障可以解释，再逐步增加路线与控制能力。** 证据是段级属性、自定义跟踪模式、障碍状态看门狗、异常收尾与事件录包。这也解释了为什么工程代码并不围绕一个“最先进规划算法”展开。

### 1.1 开发前必须纠正的几个认知

| 容易形成的印象 | 当前代码实际说明了什么 | 对开发的影响 |
|---|---|---|
| 两个执行节点已支持生命周期 | `PathFollowerNode` 和 `SchedulePlannerNode` 都直接继承 `rclcpp::Node`；整栈 launch 却把它们列为 lifecycle manager 的管理对象 | 存在源码与 launch 的集成不一致；不能仅据 launch 注释认定可以 configure/activate |
| 路网服务会自动找到任意两点间的路径 | `resolveNavPointsCallback` 对多点请求逐对调用 `findDirectEdge`，缺少直接边即失败；同时实现起点吸附/归位策略 | 增加图搜索是一项新能力，不能只把当前接口换个名字 |
| 两种执行模式只是传输方式不同 | `controller` 模式把轨迹转换成 `nav_msgs/Path`，没有把全部段属性传给控制器；也不经过 BT navigator | 不能承诺相同的全向、倒车、段限速及恢复行为 |
| 使用 Ruckig 就保证最终输出满足全部 jerk 约束 | 整段规划之后还有速度包络、PID 修正、障碍步进和外部 smoother | 必须在最终速度输出上验证约束，不能用组件名称替代端到端证明 |

源码入口：[Follower 类声明](/home/dex/dexe-mobile-nav/path_follower/include/path_follower/path_follower.hpp:58)、[Schedule 类声明及模式说明](/home/dex/dexe-mobile-nav/nav2_schedule_planner/include/schedule_planner/schedule_planner_node.hpp:33)、[路网解析](/home/dex/dexe-mobile-nav/map_asset_manager/src/route_graph_manager/route_graph_manager.cpp:888)。生命周期差异也纠正了上一份总体架构文档中把 launch 意图直接表述为已实现行为的结论。

## 二、为什么这样设计：六项值得保留的决策

### 2.1 让段携带执行语义，减少隐式上下文

**实现事实：** `TrackingPath` 同时携带几何、运动模式、倒车、保持朝向、限速、padding 和终点朝向；路径点 orientation 表示行进方向的切向，终点车头朝向另用 `target_yaw` 表达。

**为什么有价值：** 同一条几何直线，车可以前进、倒车、横移，或边移动边转向。几何本身不足以表达执行意图。把这些信息放进同一消息，可以一起记录、回放和检查，减少“路径属于任务 A，旁路参数还停在任务 B”的问题。

**代价：** 自定义接口把生产者与消费者绑在同一套语义上。迁移到标准 `FollowPath` 时需要能力映射，不能简单丢掉自定义字段。

**开发约束：** NaN 是协议的一部分，不能随意改为零。尤其 `target_yaw=NaN` 意味着不做末段旋转，`target_yaw=0` 则是明确对齐到零弧度；两者的机器人动作不同。

来源：[TrackingPath 契约](/home/dex/dexe-mobile-nav/nav_interfaces/msg/TrackingPath.msg:1)。注意消息自描述不代表执行原子性：padding 还要由 planner 单独调用 monitor 服务落实，见第四章。

### 2.2 把参考轨迹和实时速度状态分开

**实现事实：** `VelocityPlanner` 在接收路径时计算整段速度参考，运行期间保持参考曲线只读。遇到障碍后，`stepVelocityTowards` 根据上一帧速度向当前目标速度递推，不再改写轨迹数组。

相关注释明确记录了旧方案的问题：反复规划可能让曲率减速重复叠加、停车截断值污染后续规划，以及某些 Ruckig 边界输入不收敛。

**设计意义：** “在这条路径上正常应该开多快”属于参考计划；“这一刻因为障碍实际允许开多快”属于执行状态。两者分开后，障碍解除能回到同一份参考，而不是在已经降速的参考上再降速。

**代价：** 障碍减速不再与全路径速度优化统一求解。在线步进模型与静态包络之间的衔接、停车距离和恢复过程需要单独验证。

**开发约束：** 不要为临时避障直接修改 `route.points` 的参考速度。新约束应先判断是“整段恒定约束”还是“运行时可变约束”。

来源：[VelocityPlanner 设计说明](/home/dex/dexe-mobile-nav/path_follower/include/path_follower/velocity_planner.hpp:54)、[速度步进模型](/home/dex/dexe-mobile-nav/path_follower/include/path_follower/velocity_step_model.hpp:14)。

### 2.3 用解析几何支持控制，而不是不断加密路径

**实现事实：** 跟踪器会检查输入切向是否可信；可信时按切向差分求曲率，否则回退到位置几何估计。参数中保留 `legacy` 与 `toppra` 两种速度包络，当前 YAML 选择 `toppra`，但结构体默认仍是 `legacy`。

**设计意义：** 在很密的离散点上，由位置再次求导会放大误差。上游已经知道 Bezier 的几何信息，向下游传解析切向，比让下游重新从带误差的位置估计更合适。

**代价：** 下游开始依赖上游对 orientation 的定义。字段类型兼容并不足够，消息发布方必须遵守切向协议。当前角加速度约束还受曲率导数离散误差影响，代码用余量系数应对，并未完全消除误差。

**开发约束：** 路径点 orientation、车体朝向和终点朝向应分别处理。新增路径生成器时，先验证直线、圆弧和角度跨越 ±π 的情况。

来源：[朝向可信度判断](/home/dex/dexe-mobile-nav/path_follower/src/path_follower.cpp:768)、[速度包络参数及误差说明](/home/dex/dexe-mobile-nav/path_follower/include/path_follower/velocity_planner.hpp:31)、[运行配置](/home/dex/dexe-mobile-nav/path_follower/config/path_follower.yaml:128)。这里的 `toppra` 是本项目的模式名称，不应直接等同于接入了外部 TOPPRA 软件包。

### 2.4 让障碍检测输出状态，由跟踪器决定如何减速

**实现事实：** 在 `path_follower_with_collision_and_smoother.launch.py` 的接线中，monitor 的限速 Twist 输出悬空；follower 消费 `ObstacleMonitorStatus`，自己实现减速、停止、迟滞和恢复。

**为什么如此：** follower 知道当前运动阶段和上一帧输出；monitor 知道区域内是否有障碍。统一由 follower 处理障碍速度变化，可以避免两处按不同模型对同一障碍重复降速。

**代价：** 这条接线下，monitor 的 STOP 不是绕过 follower 的独立最终速度拦截。它依赖 follower 接收状态并执行处理。状态超时看门狗解决的是“follower 正常运行但监控信息中断”，不能证明 follower 自身卡住时也会及时停车。

**开发约束：** 配置、普通障碍减速、状态失效停车、底盘命令超时应分别说明。是否有独立底盘超时/急停保障，需要外部驱动和整机证据，本仓库无法确认。

来源：[旁路接线说明](/home/dex/dexe-mobile-nav/path_follower/launch/path_follower_with_collision_and_smoother.launch.py:1)、[障碍状态看门狗](/home/dex/dexe-mobile-nav/path_follower/src/path_follower.cpp:1382)。

### 2.5 跨域操作放在编排层，数据修改留在数据所属模块

重置地图原点涉及 SLAM 文件与语义资产的共同变换。如果资产管理器直接链接 SLAM 算法库，每次更换定位后端都会影响资产层。

项目设计文档选择由 `command_board` 编排，资产管理器提供 `transform_assets`，SLAM 文件由外部工具处理。资产模块只需要理解二维刚体变换，无需理解 pbstream 内部结构。

**代价：** 包内依赖更清晰，但跨进程失败处理更复杂。SLAM 文件修改成功而资产变换失败时，编排者必须知道系统处于哪一步；简单返回 `false` 无法自动恢复一致性。

**开发约束：** “能调用服务”不是把编排塞进任意模块的理由。谁启动跨域操作，谁应负责记录阶段、汇总结果和决定失败后的处理。

来源：[原点重置方案与取舍](/home/dex/dexe-mobile-nav/map_asset_manager/docs/reset_map_origin_design.md:1)、[实际编排实现](/home/dex/dexe-mobile-nav/command_board/apps/command_board_ros.cpp:120)。方案文档中的所有流程仍需与实现核对。

### 2.6 让取证工具依赖导航事件，避免导航依赖录包器

planner 用 `std_msgs/String` 发布触发原因；录包模块负责窗口、存储、保留策略和日志。自动触发源通过 `TriggerSource` 扩展，常态流和事件流通过 `RecordingSink` 区分。

**设计意义：** 导航代码只声明“此处值得留证据”，不用等待磁盘写入，也不用知道 bag 文件如何组织。增加一种故障触发源通常不需要改录制器主体。

**代价：** topic 触发没有业务确认，无法承诺事件一定落盘。当前源码中，限速只影响常态流，事件流收全量；README 后部仍有“两条流都限速”的旧说明，应以实现为准。

来源：[触发接口](/home/dex/dexe-mobile-nav/mobile_bag_recorder/include/mobile_bag_recorder/trigger_source.hpp:26)、[消息分发与限速](/home/dex/dexe-mobile-nav/mobile_bag_recorder/src/topic_subscriber_set.cpp:133)、[planner 触发点](/home/dex/dexe-mobile-nav/nav2_schedule_planner/src/schedule_planner_node.cpp:638)。

## 三、职责边界：谁拥有数据，谁拥有决策

### 3.1 用所有权判断新功能应该放在哪里

| 组件 | 当前拥有的状态/决策 | 应保持的边界 | 扩展时常见的越界 |
|---|---|---|---|
| `command_board` | 系统命令、跨域流程、地图版本协调 | 调用各域能力并处理跨域结果 | 直接实现轨迹控制或几何算法 |
| 资产 Store / FileManager | 资产记录、查找、序列化与文件写入 | 存储形式由资产域封装 | 控制节点直接修改 JSON |
| 资产 Manager / RouteGraphManager | CRUD 服务、引用与解析策略；当前也读机器人位姿做吸附/归位 | 对外提供资产语义，内部协调 store | 把实时避障或电机状态混入资产模型 |
| `schedule_planner` | 当前任务、段序列、合并、段级参数生效顺序、超时与终态 | 决定执行哪段及何时结束任务 | 在等待反馈时另做一套实时控制 |
| `path_follower` | 当前执行轨迹、控制阶段、速度状态、定位输入有效性 | 计算本周期指令并报告执行状态 | 接管业务任务队列、地图版本或路网编辑 |
| `collision_monitor` | 传感器源、区域规则、障碍判定及其状态 | 对外明确检测状态与时效 | 空闲保活主动产生运动命令 |
| `mobile_bag_recorder` | 缓冲、触发窗口、录制生命周期、磁盘保留 | 独立完成取证 | 为录制成功而阻塞控制循环 |

**边界并非已经完全纯净。** 例如 RouteGraphManager 既管理静态资产，也做依赖实时位姿的起点选择；这是为了集中实现“点名到可执行段”的便利。以后需要支持不同归位策略或离线规划时，这部分会成为自然的拆分点。

### 3.2 不要把三类状态混成一个状态机

- **任务状态：** 当前用户请求是否完成，由 planner 的 action 管理。
- **运动状态：** 当前段正在旋转、跟踪、减速或停止，由 follower 管理。
- **检测状态：** 当前区域是否存在障碍，由 monitor 管理。

例子：障碍 STOP 可以先让 follower 停车等待，不等于任务应立即 ABORTED；但障碍状态长时间未更新，当前 follower 实现会停止并报告 FAILED。前者是环境阻碍，后者是控制输入不可信。新增状态时必须明确它如何向上层传播。

### 3.3 同进程事件总线与 ROS 通信解决不同的问题

`DataEventBus` 同步通知资产联动，采用锁内复制回调列表、锁外执行回调，避免执行用户回调时持有总线锁。这样 point、route、空间索引等不必直接互相调用。

但它没有持久化事件日志、自动重试或回滚；回调慢会延长调用者耗时，某个回调失败也不能被解释为“所有资产更新自动撤销”。如果为事件回调新增文件写入，应同时说明失败时内存、索引、磁盘各处的状态。

来源：[DataEventBus 实现](/home/dex/dexe-mobile-nav/map_asset_manager/src/data_event_bus.cpp:10)。

## 四、开发者已经解决的工程问题，以及保证的边界

### 4.1 取消既是运动操作，也是 action 状态转换

**原始问题：** “让机器人停下”与“把 action 标为 CANCELED”不是同一个操作。外部 force-cancel 服务触发时，action 可能仍处于 EXECUTING。

**现有处理：** 正式 action cancel 进入取消流程后返回 CANCELED；force-cancel 通过停车流程结束任务，但把 action 标为 ABORTED，避免非法状态转换。controller goal 句柄通过作用域收尾清空，工作线程入口统一捕获异常，防止异常逃出 `std::thread` 导致节点退出。

**开发经验：** 每条失败路径都要分别处理“运动应如何结束”和“上层请求应如何结束”。调用停车服务、收到停车应答与底盘物理停止也是不同层次，当前任务结果不能替代物理停止测量。

来源：[任务收尾](/home/dex/dexe-mobile-nav/nav2_schedule_planner/src/schedule_planner_node.cpp:391)、[线程异常边界](/home/dex/dexe-mobile-nav/nav2_schedule_planner/src/schedule_planner_node.cpp:216)、[controller 句柄清理](/home/dex/dexe-mobile-nav/nav2_schedule_planner/src/schedule_planner_node.cpp:730)。

### 4.2 参数修改必须先于依赖它的动作

新段可能要求不同停障距离。planner 先等待 `set_polygon_padding` 返回成功，再发布轨迹；默认服务失败就拒绝执行该段，避免带着上一段的停障距离起步。

这是一个很小但关键的顺序保证：**先确认前置条件，再触发动作。** 把调用改为 fire-and-forget，会改变系统行为。

保证也有边界：多个调用方共享同一个 polygon 参数时，服务成功之后仍可能被其他调用者覆盖；任务结束后的默认 padding 恢复当前被注释掉，monitor 会保留最后一段的值。以后引入多个执行器，要处理参数所有权，不能仅增加一把进程内锁。

来源：[runSegment](/home/dex/dexe-mobile-nav/nav2_schedule_planner/src/schedule_planner_node.cpp:449)、[未启用的恢复逻辑](/home/dex/dexe-mobile-nav/nav2_schedule_planner/src/schedule_planner_node.cpp:429)。

### 4.3 安全状态必须在机器人不动时也有意义

若 monitor 只在收到速度指令时发布状态，follower 在起步前无法区分“监控未运行”和“只是没有速度”。idle keepalive 用零速度评估并发布障碍状态，且禁止这条路径发布 Twist。

保活只更新历史 action type，不覆盖历史请求速度；否则下一次真实零速指令可能丢失“刚刚停下”的边沿。运动期间有速度输入时，保活跳过，避免重复评估。

**开发经验：** 看门狗需要独立、可解释的更新节奏；保活也不能顺手改写业务状态。

来源：[idleKeepaliveCallback](/home/dex/dexe-mobile-nav/nav2_collision_monitor/src/collision_monitor_node.cpp:278)、[process 两条路径的状态更新](/home/dex/dexe-mobile-nav/nav2_collision_monitor/src/collision_monitor_node.cpp:585)。

### 4.4 定位准确性与控制连续性需要不同处理

follower 实现了 map→odom 跳变检测、odom 递推/等待恢复、末端定位锚定，以及独立的 odom 接收超时检查。它没有实现一个新的 SLAM 或通用状态估计器；这里是在决定“当前控制周期应该相信哪份位姿，以及是否还能继续执行”。

**取舍：** 末端锚定可减少全局定位抖动引起的反复纠偏，但锚定期间更依赖 odom；odom 连续也不代表全局位置正确。调高阈值会减少停车，同时可能扩大容忍的定位误差，因此应结合目标误差和跳变现场，而不是只追求少报错。

来源：[相关参数与语义](/home/dex/dexe-mobile-nav/path_follower/src/path_follower.cpp:354)、[跳变处理](/home/dex/dexe-mobile-nav/path_follower/src/path_follower.cpp:1758)。

### 4.5 等待外部结果不能占住所有回调

planner 用工作线程执行长任务，executor 继续收 action/service/status 回调。follower 的舵角预摆通过独立 helper node 等待结果，但这段同步等待仍占用 follower 所在的单线程 executor。

**现有收益：** helper node 能处理被等待的 action 通信，避免依赖原节点同一执行器来完成响应。

**剩余代价：** 这段时间 follower 的控制 timer 和普通订阅仍不能正常推进。延长某个超时只是增加等待容忍度，无法恢复回调实时性；并且当前普通 Node 没有 launch 注释所描述的 lifecycle bond 能力。

**建议：** 优先把预摆建模为异步准备阶段；如果改多线程 executor，还必须明确 callback group、状态所有权和锁顺序。增加线程数本身不是完整修复。

来源：[舵角同步等待](/home/dex/dexe-mobile-nav/path_follower/src/path_follower.cpp:1130)、[planner 执行线程约定](/home/dex/dexe-mobile-nav/nav2_schedule_planner/include/schedule_planner/schedule_planner_node.hpp:42)。

### 4.6 保存文件、统一备份和事务是三个层次

资产保存先写临时文件再 rename，避免直接覆盖正式 JSON 时留下半写文件。但代码没有显式文件/目录 `fsync`，不能据此承诺断电后的持久性。

`OnBackupUnified` 顺序调用地图备份与资产备份，返回两个版本号；第二步失败时，第一步备份已经发生。回退也是逐个文件执行 `AtomicCopy`。**单文件原子替换没有把跨文件、跨服务操作变成一个事务。**

**开发经验：** 对持久化功能分别说明：单文件是否完整、多个文件是否同版本、进程崩溃后是否可恢复、断电后是否持久。不要都用一个“原子安全”概括。

来源：[资产原子替换](/home/dex/dexe-mobile-nav/map_asset_manager/src/file_manager.cpp:665)、[统一备份与回退](/home/dex/dexe-mobile-nav/command_board/src/map_version_manager.cpp:138)。

### 4.7 现场观测应该围绕故障窗口设计

录包器用独立完整 bag 做分段回收；事件流保留触发前后的时间窗口；QoS 按发布端适配；存储后端做启动自检；日志采集与实时消息处理分离。它解决的是“故障发生时，是否还能找到之前的数据”。

开发时要保留两个维度：常态录制可以降频控制成本，事件流应按明确策略保留细节。时间窗口仍可能受缓冲容量限制，触发消息也可能丢失，不能把功能描述理解为无条件留证成功。

来源：[录包设计说明](/home/dex/dexe-mobile-nav/mobile_bag_recorder/README.md:1)、[事件流绕过限速](/home/dex/dexe-mobile-nav/mobile_bag_recorder/include/mobile_bag_recorder/event_recorder.hpp:62)。

## 五、稳定层与易变层：按变化成本管理

这里的“稳定”指**应该保持兼容的设计面**，不是依据提交历史统计的变更频率。当前工作区没有可用 Git 历史用于这种统计。

| 层面 | 稳定性判断 | 变化为什么昂贵 | 建议的变更方式 |
|---|---|---|---|
| 坐标系、单位、切向/车体朝向定义 | 应最稳定 | 破坏后通常表现为运动错误，而非编译错误 | 写清契约，并用已知几何实例验证 |
| action/msg/srv 的字段语义 | 应稳定，但当前仍有缺口 | 发布者、消费者、脚本、bag 与外部部署都受影响 | 统一发布版本；跨版本部署考虑新接口或适配层 |
| 资产 ID、引用规则、文件 schema | 应稳定 | 现场已有地图和配置是长期资产 | 提供格式迁移、旧数据读取与回退方案 |
| 取消、停车、失效上报语义 | 应稳定 | 上层恢复流程与操作员判断依赖这些语义 | 用状态转换和故障场景证明兼容 |
| 采样、合并、速度包络、PID 策略 | 易变化 | 底盘、场地和精度目标推动调整 | 通过策略或纯函数替换，保留对外契约 |
| launch、topic、namespace、参数值 | 经常随部署变化 | 属于系统接线，错误可能静默发生 | 将参数实际值和依赖节点纳入集成检查 |
| 录包触发源、保留策略、展示工具 | 易扩展 | 现场问题和产品交互不断增加 | 利用已有 TriggerSource/RecordingSink 等扩展点 |
| `nav2_costmap_2d`、`nav2_util` 等 fork | 上游接口较稳定，本地差异需单独管理 | 升级容易覆盖本地行为，系统库与工作区库可能混用 | 记录本地补丁目的，分别验证上游兼容与本地需求 |

**不要把 `nav_interfaces` 的中心地位误解为“所有新消息都往这里放就没有耦合”。** 它稳定的是共享语义，同时也是变更传播范围最大的地方。字段增加、默认值改变或枚举重用，都需要检查已有生产者和消费者。

## 六、当前最值得正视的设计债务

### 6.1 段状态缺少关联标识

`NavigationStatus` 只有 `status` 和 `message`，没有任务 ID、段 ID 或时间戳。planner 发新段前清空本地状态，但 `onNavigationStatus` 无条件接受随后到达的状态。

因此“清空上一段状态”只能清除本地残留，不能识别随后晚到的上一段 FINISHED；直接发布 `/tracking_path` 的其他调用方也没有明确执行权隔离。这是协议层面的风险，不是已复现故障的断言。

**建议：** 引入执行 ID、段序号和明确的接受应答，或定义携带 `TrackingPath` 的段执行 action。先补关联语义，再考虑多个任务并发或复杂抢占。

来源：[NavigationStatus](/home/dex/dexe-mobile-nav/nav_interfaces/msg/NavigationStatus.msg:1)、[发布前复位](/home/dex/dexe-mobile-nav/nav2_schedule_planner/src/schedule_planner_node.cpp:489)、[状态消费](/home/dex/dexe-mobile-nav/nav2_schedule_planner/src/schedule_planner_node.cpp:1016)。

### 6.2 “连续段可合并”实际上是一套业务规则

`sameTrackingAttributes` 检查 backward、padding、解析后的 mode；并未要求全部段属性相等。采样器对组内 `max_speed` 取最小值，`hold_direction` 采用首段值。这些是合并后的行为定义，不是单纯的几何优化。

例如两段限速分别 0.4 与 0.8 m/s，合并后整组按 0.4 m/s；两段全向平移要求不同保持朝向时，采用首段值可能违背后段意图。新增字段时，应明确它是“必须相等”“取最保守值”“按区间保留”还是“禁止合并”。

当前 planner 还用“段数等于点数”推断存在归位段。增加多跳图搜索后，这个数量关系可能失效，应改为显式段类型或来源标记。

来源：[合并条件](/home/dex/dexe-mobile-nav/nav2_schedule_planner/src/segment_merger.cpp:66)、[合并属性处理](/home/dex/dexe-mobile-nav/nav2_schedule_planner/src/path_segment_sampler.cpp:541)、[归位段推断](/home/dex/dexe-mobile-nav/nav2_schedule_planner/src/schedule_planner_node.cpp:295)。

### 6.3 两种后端有共同接口，但没有完整能力协商

共用任务外壳可以复用取消和结果流程，这是合理的过渡设计。但 controller 模式转换成标准 Path 时，许多段属性没有传递，合并也被关闭。当前外层 action 并没有据此形成完整的“不支持能力”拒绝协议。

**建议：** 给后端定义能力集合，例如是否支持倒车语义、全向保向、分段限速、动态 padding。执行前校验，无法保持语义时明确拒绝或返回降级说明。

### 6.4 自动测试覆盖与核心风险不对称

资产管理和录包器有针对 store/service、缓冲与保留策略的测试；本次检查中，`path_follower` 与 `nav2_schedule_planner` 的 CMake `BUILD_TESTING` 部分仅注册 lint，存在演示脚本但未见同等级的自动行为测试注册。

优先补的应是语义测试：晚到段状态、取消与结果竞争、缺失障碍状态、倒车切向、NaN 终点朝向、合并属性冲突、定位跳变与恢复。它们比逐个断言 getter 或重写实现逻辑更能保护开发。

这只是仓库内测试入口的核查，不能据此否认仓库外的整机测试。

## 七、如果重新设计：保留什么，替换什么

以下都是演化建议，不代表当前已实现。

| 议题 | 我会优先采用的设计 | 收益 | 成本与选择条件 |
|---|---|---|---|
| 段执行协议 | 自定义段 action 或带执行 ID 的请求/应答协议 | 接受、反馈、结果、取消有明确归属 | 接口迁移与客户端改造；仍需自己处理抢占/迟到结果 |
| follower 内部组织 | ROS 适配层 + 准备状态机 + 可独立测试的控制核心 | I/O 等待与周期计算分开，复现更容易 | 需要把当前共享状态与时间源显式化 |
| 速度与安全 | 正常速度策略仍由 follower 统一处理；另评估最终独立失效停车门控 | 保留平滑性，同时缩小 follower 故障影响 | 避免普通障碍重复制动；需整机测量延迟、距离及底盘超时 |
| 路网解析 | 静态图存储与“起点选择/搜索/归位”策略分离 | 支持离线规划、不同起点策略和未来图搜索 | 当前直接边场景下会增加抽象层 |
| 轨迹消息 | 明确切向可信标志；确有需求时传曲率/导数、区间约束 | 减少重复估计和合并信息损失 | 消息体积、生成器责任和版本管理成本增加 |
| 地图版本 | 用 manifest 绑定地图、资产版本及校验信息；记录流程阶段 | 能解释部分失败并选择一致版本恢复 | 增加磁盘协议与恢复逻辑；还需定义执行中的编辑规则 |
| 本地 Nav2 改造 | 插件能表达的行为做插件；必须改核心的维护小补丁集 | 缩小升级冲突范围 | 插件边界之外的行为仍需 fork，不能强行插件化 |

### 7.1 不建议一次性重写的部分

保留段级契约、只读参考曲线、资产域隔离和非阻塞故障触发。这些设计已经直接对应可解释的工程问题。

也不必立即把所有 manager 拆成进程，或为一次性事件增加分布式消息系统。新增进程会带来时序、版本与失败协调成本；当前同进程 store/event bus 对资产联动是可理解的选择。

### 7.2 建议的实施次序

1. **先修正集成事实：** 对齐普通 Node 与 lifecycle launch 的不一致，明确哪些节点实际可被管理；修正文档与运行参数的偏差。
2. **再封住协议漏洞：** 段结果关联、执行权、取消终态、后端能力和合并属性语义。
3. **然后改善控制可测试性：** 异步准备阶段、显式时间源、纯控制/几何测试与故障注入。
4. **最后扩展产品能力：** 多跳寻路、复杂任务抢占、地图一致版本与更丰富的轨迹约束。

这个次序的原因是：协议含糊时，增加功能会增加更多难以关联的状态；先澄清状态与责任，后续算法改造才有可靠的验证边界。

## 八、从读代码到提交改动：三个真实开发练习

### 8.1 需求：某段需要更低限速

先确认是“写入地图的段属性”还是“执行时临时约束”。前者检查 asset → PathSegment → TrackingPath → follower 的字段链；后者还需要明确生效时间、优先级与解除条件。

必须检查合并后取最小限速的行为，以及 controller 模式不会自动继承该字段。至少验证单段、合并段、NaN 默认、倒车、障碍解除后的恢复。调一个全局 YAML 上限，只解决所有段一起变慢的场景。

### 8.2 需求：自动绕过被占用的路线边

当前相邻点解析依赖直接边，monitor STOP 也不会自然变成全局绕路。需要新增边可用性来源、图搜索、路线替换和旧任务取消的完整语义。

资产域保留静态连接；动态占用信息应有自己的时效；规划策略组合两者；执行器按新任务/段 ID 接管。不要把实时传感器障碍永久写进静态 JSON，也不要只在 `findDirectEdge` 失败时随便挑一条边。

### 8.3 需求：新增一个现场异常录包触发源

在 `TriggerSource` 实现中检测异常边沿，在 builtin 清单和构建文件中登记，并补参数。录制器继续负责冷却、并发抑制和窗口。

验证持续异常只触发一次、恢复后再次异常可触发、禁用时不建立不必要订阅，以及录制不可用时不影响导航。这个练习修改范围较小，适合作为熟悉项目扩展方式的第一项任务。

### 8.4 评审时应能给出的五个答案

- **职责：** 为什么功能放在这个模块？它新增了什么依赖？
- **契约：** 输入语义、默认值、状态转换是否改变？哪些消费者需要同步更新？
- **时序：** 等待发生在哪个线程？超时、取消、重复或晚到结果如何处理？
- **退路：** 部分成功后剩下什么状态？如何恢复或明确失败？
- **证据：** 哪些可观察结果说明需求满足，同时没有破坏已有停车和任务语义？

能够清楚回答这五点，就已经开始以项目开发者的方式工作：算法是实现手段，契约、状态与可验证行为决定改动能否长期维护。

---

**核查范围：** 本手册结合现有 Graphify 索引定位相关符号，并回到源码确认行为；未重新构建整个图。本文没有修改运行代码，也没有启动机器人、执行运动或运行 ROS 集成测试。关于迟到状态、跨文件一致性和独立停车边界的内容，是静态分析发现的限制及风险，需要相应测试确认具体现场表现。
