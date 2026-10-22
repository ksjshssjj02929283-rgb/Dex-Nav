# 迦智遥控接管与整机故障停车

## 控制链与职责

```text
mobile_joy_control -- /control/cmd_vel_cloudia_joy_raw --> command_board
command_board      -- /control/cmd_vel_cloudia_joy     --> 原 cmd_vel_mux
其他速度来源 -----------------------------------------> 原 cmd_vel_mux
cmd_vel_mux        -- /control/cmd_vel                --> Server WebSocket 适配器
```

默认 `chassis_backend=dexe`。自研差速、全向的话题和手柄算法保持原行为。
`cloudia` 模式下，手柄的正常速度、零速度都只发到 `_raw`；mux 不订阅 `_raw`。
迦智遥控入口优先级为 10，高于原有最高值 9。其他入口的配置未改变。
最终速度话题不应有绕过 mux 的生产发布者。

`CloudiaTeleopManager` 是 `CommandBoardManagerROS` 持有的 C++ 组件，不是
独立进程。它使用 libcurl 直接请求迦智 HTTP；Server 持有唯一 WebSocket。
HTTP 在后台线程运行；ROS 回调处理手柄输入、安全状态和速度发布。
组件不依赖 Python RobotClient，不与 Client/application 进行任务提交申请或结果回报，
也不重写摇杆、滤波或软停车算法。

复用 `/control/joy_in`：`1` 同时表示使能和松键减速；`0` 表示释放；
`-1/-2` 不允许新的遥控放行。没有新增手柄状态话题。
如果松键发生在等待停止任务期间，只要 `joy_in` 仍为 `1`，确认后仍按原软停车
语义转发新收到的减速速度。只转发接管确认之后收到的最新帧；不重放旧缓存。

## 接管与任务边界

正常接管状态为 `IDLE → STOPPING_TASK → TELEOP → RELEASING → IDLE`。
请求接管后重新读取远端状态；若 8002 网关报告 `taskStatus=running`，调用
`POST /api/v1/task/stop`。只有后续状态显示网关无运行任务、设备就绪且速度接近零，
才允许转发新的手柄速度。HTTP `code=0` 只表示停止请求被受理，不代表已经停车。
等待停车超过配置时限、输入丢失或状态失效会保持零速并进入保护状态。

不提供任务登记、许可、回报或任务队列，不主动阻止 application/Client 下发新任务。
若遥控期间观察到新的网关运行任务，会撤销非零转发并尝试停止该任务。
这里依赖状态轮询，不能保证在途 HTTP 请求和遥控接管之间具有原子互斥。

8002 网关的 task 与 8858 调度中心的 mission 是不同的任务系统。
网关 `task:null` 不表示调度中心没有 mission。本组件不取消、暂停、恢复或重发调度
mission；调度任务可以在遥控速度占优期间保留，并在速度通道释放后继续执行，
实际行为由迦智内部仲裁和调度决定。被 8002 stop 真正终止的任务不由本组件自动恢复。

旧版任务登记文件及其锁文件不再读取、写入或创建；升级不会自动删除这些旧文件。

## 普通导航停障与控制权释放

普通导航停障单独标记为 `navigation_blocked`，不再直接归入故障。
只有导航状态（IDLE/RUNNING/WARNING）中的 `obstacleStop`、且无急停或碰撞时才按
普通停障处理；两个 HTTP 状态任一报告 ERROR 等其它状态时仍保留保护，避免采样先后
差异掩盖故障。初始化、充电和其它未识别状态的原有处理不在本次变更范围内。

- 正常待命且未持有速度控制时，普通停障不触发持续零速，也不因此调用 8002 stop。
  迦智导航自身负责停障和恢复；日志记录普通停障状态的出现与退出。
- 停障期间拒绝新的遥控接管。使能键若已按下，必须松开再重新申请，避免标志解除后
  自动执行一直保持的摇杆输入。
- 已进入接管或遥控时发生停障，立即停止非零转发，进入释放阶段。
  只有上半身安全、最新远端反馈无硬故障且已静止、网关无运行任务并松开使能键后，
  才发送有限零速尾帧并释放通道；不会把普通停障期间的新鲜非零手柄消息继续转发。
- 硬故障解除后若只剩普通停障，上述条件满足时也可以释放零速通道，不再要求先等到
  `informationType=1`。此条件仅允许归还控制权，不允许在停障中放行遥控。

停障解除后仍需经过正常接管流程，并只转发放行后新收到的速度。
本组件不清除迦智停障标志，不改变迦智避障参数，也不调用调度 mission 取消接口。

## 故障停车

现有 StatusMonitor 接收整机状态，OnRecoverTimer 继续判断故障。
Cloudia 组件在首次有效整机状态之前、收到非就绪状态、状态失效，
或安全检测心跳超时后，禁止非零转发。自研底盘原有安全/恢复条件未改变。

故障先立即占用 mux 的高优先级入口发零速度，并唤醒 HTTP 停止工作。
随后持续核对远端状态；观察到网关任务仍在运行时重复请求停止。
故障解除且确认静止后，手柄需释放，才回到待命；仅剩普通导航停障时也允许按上述条件归还通道。不会重放故障前速度。
停止失败或状态不可用时持续零速度并报错，不能声称已经停稳。
故障停车不等于取消调度 mission：高优先级零速度可压住底盘运动，但调度任务仍可能保留。

现有 StopRobotMovement() 入口也通知此组件。地图、定位恢复和传感器组件
原有职责保留；没有把故障检测复制到迦智组件。

正常待命时本组件不持续发布零速度；松键释放后只有有限零速尾帧。
应搭配已修复的 Server 适配器：无速度输入时只接收状态，输入超时发送有限零速后
停止刷新 WebSocket 速度通道；持续的新鲜故障零速度仍正常转发。
WebSocket 断开不等于底盘自动停车，网络完全断开后的停车仍依赖底盘及整机保护。

## 构建与启动配置

需要有效的 ROS Humble、项目原有 dexe_interfaces/nav_interfaces/chassis_interfaces
以及 libcurl 开发包。无需构建或部署额外的任务登记接口包，也不需要同步编译 Client。

```bash
colcon build --packages-select command_board mobile_joy_control cmd_vel_mux
source install/setup.bash
```

在启动进程的环境中设置：

```bash
export DEXE_CHASSIS_BACKEND=cloudia
export CLOUDIA_HOST=<迦智工控机地址>
# CLOUDIA_TOKEN 由现场凭据管理提供，不写入仓库或日志
```

`command_board.launch.py`、`joy_only.launch.py` 均接受 `chassis_backend:=cloudia`，
默认取上述环境变量，否则为 dexe。命令板、手柄与 Server 需要选择一致的后端。
单独 ros2 run 配参数文件时，需要显式覆盖 `chassis_backend`，避免 YAML 中的
`dexe` 默认值覆盖环境默认值。

HTTP、输入和状态超时、静止阈值、话题名称位于
`params/command_board_params.yaml` 的 `cloudia` 参数组。
`cloudia.host` 未配置或为空时读取 `CLOUDIA_HOST`。

## 验证范围

`cloudia_teleop_state_test` 覆盖接管、遥控放行、软停车、超时和故障保持；
`cloudia_teleop_manager_test` 使用 127.0.0.1 假网关覆盖任务停止、速度转发和故障重复停车。
运行联调测试应指定独立 ROS_DOMAIN_ID 和 ROS_LOCALHOST_ONLY=1。
这些检查不代表实机停车距离、网络故障停车或导航/交管业务重试策略已经验收。
