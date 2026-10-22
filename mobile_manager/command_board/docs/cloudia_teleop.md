# 迦智遥控接管与整机故障停车

## 控制链与职责

```text
mobile_joy_control -- /control/cmd_vel_cloudia_joy_raw --> command_board
command_board      -- /control/cmd_vel_cloudia_joy     --> 原 cmd_vel_mux
其他速度来源 -----------------------------------------> 原 cmd_vel_mux
cmd_vel_mux        -- /control/cmd_vel                --> 原 Server WebSocket 适配器
```

默认 `chassis_backend=dexe`。自研差速、全向的话题和手柄算法保持原行为。
`cloudia` 模式下，手柄的正常速度、零速度都只发到 `_raw`；mux 不订阅 `_raw`。
迦智成品速度入口优先级为 10，高于原有最高值 9。其他入口的配置未改变。
最终速度话题不应有绕过 mux 的生产发布者。

`CloudiaTeleopManager` 是 `CommandBoardManagerROS` 持有的 C++ 组件，不是
独立进程。它使用 libcurl 直接请求迦智 HTTP；Server 仍持有唯一 WebSocket。
HTTP 在后台线程运行；ROS 回调处理输入、登记、状态和速度发布。
组件不依赖 Python RobotClient，也不重写摇杆、滤波或软停车算法。

复用 `/control/joy_in`：`1` 同时表示使能和松键减速；`0` 表示释放；
`-1/-2` 不允许新的遥控放行。没有新增手柄状态话题。
如果松键发生在等待停止任务期间，只要 `joy_in` 仍为 `1`，确认后仍按原软停车
语义转发新收到的减速速度。只转发接管确认之后收到的最新帧；不重放旧缓存。

## 任务提交协调

新增接口包 `cloudia_control_interfaces`：

* `/command_board/cloudia/request_task_submission`：Client 生成唯一 request_id，
  command_board 先持久化登记，再返回允许和 coordinator_id。
* `/command_board/cloudia/report_task_submission`：Client 回报 ACCEPTED（含 taskId）、
  REJECTED、UNCERTAIN 或 NOT_SENT。结果回报可重试，HTTP 启动请求不能自动重试。

所有迦智生产任务必须使用更新后的 `CloudiaTaskClient.start()`。
旧 Client 或直接请求迦智 HTTP 的程序不受本协调机制约束。
每次仅允许一笔在途提交；仍在执行的远端任务也不接受新的提交许可。
Client 协调通信使用私有 ROS executor，避免同步 start() 在调用方的单线程
回调中等待自身响应。对外 ChassisClient/W1Manager 方法签名不变。

接管先关闭新申请，等待已登记 HTTP 提交明确结果，再停止当前远端任务。
这里等待的是提交受理结果，不是导航执行完成。HTTP `code=0` 只是受理；
允许遥控还需新的任务状态查询和实际速度反馈证明任务结束、设备就绪且静止。
收到 ACCEPTED 回报后，先确认该 taskId 出现在远端状态中，不能把前一个
任务的终态误当作新任务已经结束。

网络超时、缺失 taskId、进程退出导致无法回报，都会保留不确定登记。
`submission_timeout_sec` 仅把未回报请求标记为不确定，绝不自动删除登记。
已被远端受理而报告 ACK 丢失时，Client 只重报结果，不重发启动请求。
`start()` 返回 True 仍表示 HTTP 已受理；不代表任务完成。
不确定的启动结果在 Client 中标记为 LOST，不能当成已明确拒绝。

## 持久登记与恢复

默认登记文件：`~/.ros/cloudia_task_submissions.json`，以原子替换和 fsync
保存；旁边的 `.lock` 文件防止两个组件同时使用同一个登记文件。
不保存 Token 或任务业务参数。恢复出的在途登记一律视为不确定。
原 Client 的迟到明确回报仍可使用旧 coordinator_id 结清登记。
登记文件损坏、不可写、无法加锁时，不能授予任务/遥控权限。

不确定登记不能仅凭一次“当前空闲”查询清除：还可能有迟到请求。
组件不提供不经确认的自动清除。必须先停止所有生产提交者，确认不存在在途
请求、网关任务和实际运动，再由维护人员处理登记并重启。运行中不得删除登记
文件来强行解除阻断。网关 taskId 重置或非受管程序插入任务，也可能需要人工核对。

## 故障停车

现有 StatusMonitor 接收整机状态，OnRecoverTimer 继续判断故障。
Cloudia 组件在首次有效整机状态之前、收到非就绪状态、状态失效，
或安全检测心跳超时后，禁止任务提交和非零转发。
自研底盘原有安全/恢复条件未改变。

故障先立即占用 mux 的高优先级入口发零速度，并唤醒 HTTP 停止工作，
不等待在途提交完成。随后持续核对远端状态；迟到任务仍在运行时重复请求停止。
HTTP 成功回调不能解除安全闭锁。故障解除且确认静止后，手柄需释放，
才回到待命。旧任务不自动恢复，也不重放故障前速度。
停止失败或状态不可用时持续零速度并报错，不能声称已经停稳。

现有 StopRobotMovement() 入口也通知此组件。地图、定位恢复和传感器组件
原有职责保留；没有把故障检测复制到迦智组件。

注意：原 Server 适配器在没有输入时仍周期发送零速度，输入超时也会发零；
本次没有修改它。自主导航与这些零速度的兼容性须按厂商协议实机验证。
文档明确 WebSocket 断开不会自动下发零速度，因此网络完全断开后的停车
仍依赖底盘本身的超时保护/整机安全措施，ROS 零速度不等于硬件急停。

## 构建与启动配置

需要有效的 ROS Humble、项目原有 dexe_interfaces/nav_interfaces/chassis_interfaces
以及 libcurl 开发包。新接口需要同时部署给 command_board 和使用 RobotClient
的机器；pip 更新 Client 不会自动安装 ROS 接口包。

```bash
colcon build --packages-select cloudia_control_interfaces command_board mobile_joy_control cmd_vel_mux
source install/setup.bash
```

在各进程的启动环境中统一设置：

```bash
export DEXE_CHASSIS_BACKEND=cloudia
export CLOUDIA_HOST=<迦智工控机地址>
# CLOUDIA_TOKEN 由现场凭据管理提供，不写入仓库或日志
```

现有 `command_board.launch.py`、`joy_only.launch.py` 均接受
`chassis_backend:=cloudia`，默认取上述环境变量，否则为 dexe。
只设置一个进程的后端会造成链路不一致。命令板必须启动；Client 无法联系到
协调服务时拒绝发送 HTTP。单独 ros2 run 配参数文件时，需要显式覆盖
`chassis_backend`，避免 YAML 中的 dexe 默认值覆盖环境默认值。

HTTP、输入和状态超时、静止阈值、接口名称位于
`params/command_board_params.yaml` 的 `cloudia` 参数组。
`cloudia.host` 未配置或为空时读取 `CLOUDIA_HOST`。
Client 可通过 `CLOUDIA_REQUEST_SUBMISSION_SERVICE`、
`CLOUDIA_REPORT_SUBMISSION_SERVICE`、`CLOUDIA_SUBMISSION_TIMEOUT_S`
覆盖协调接口和调用超时。

## 验证范围

`cloudia_teleop_state_test` 验证策略；`cloudia_teleop_manager_test` 使用
127.0.0.1 假网关验证真实 ROS 请求、停止和转发、回报幂等与登记恢复。
运行联调测试应指定独立 ROS_DOMAIN_ID 和 ROS_LOCALHOST_ONLY=1。
Client 的 `tests/test_cloudia_submission.py` 验证请求顺序和异常；
设置 `CLOUDIA_RUN_ROS_TESTS=1` 才运行单线程 ROS 回调兼容性用例。
这些测试不代表实机停车距离、网络故障停车或导航/交管业务重试策略已验收。
