# 底盘直控测试

本目录包含三个可独立运行的 Python 脚本，用于测试底盘速度响应，以及直线运动和原地转向的底盘直控效果。三个脚本彼此没有文件依赖，单独复制一个脚本也可以运行，但机器上必须具备对应的 ROS 2 接口和依赖。

## 选择测试脚本

| 想测试的内容 | 使用脚本 |
| --- | --- |
| 底盘能否准确跟随指定线速度或角速度 | `chassis_test_common.py` |
| 直线运动的耗时、最终误差、横向偏移和过冲 | `precise_straight_action_test.py` |
| 原地转向的耗时、最终误差和过冲 | `precise_rotation_action_test.py` |

`chassis_test_common.py` 直接发送速度，不负责控制运动距离或角度。另两个脚本通过 `/control/precise_control` Action 发送运动目标，用于测试 `mobile_control_action_node`。

## 第一次运行

### 1. 准备环境

每个新终端都需要加载 ROS 2 和当前工作空间：

```bash
cd dexe_mobile_nav
source /opt/ros/humble/setup.bash
source install/setup.bash
cd mobile_perception/test
```

如果提示找不到 `nav_interfaces`、`rclpy` 或其他 ROS 2 模块，通常是当前终端没有执行上述 `source` 命令。

### 2. 启动所需节点

测试直线运动或原地转向前，在另一个已经加载环境的终端启动底盘直控节点：

```bash
ros2 launch mobile_perception mobile_control_action.launch.py
```

测试速度响应时，应启动 `cmd_vel_mux` 所在的底盘速度链路，确保：

```text
/control/cmd_vel_nav -> cmd_vel_mux -> /control/cmd_vel
```

可以使用以下命令检查接口是否存在：

```bash
ros2 topic info /feedback/odom
ros2 topic info /control/cmd_vel_nav
ros2 action info /control/precise_control
```

### 3. 确认实机安全

- 停止导航、遥控和其他会发布底盘速度的任务。
- 确认急停可用，机器人周围有足够的直行或旋转空间。
- 第一次使用较小的速度、距离或角度，确认方向正确后再增大测试量。
- 线速度正值表示前进，负值表示后退；角速度或角度正值表示逆时针，负值表示顺时针。

## 速度响应测试

### 阶跃速度

以下命令让机器人以 `0.01 m/s` 前进 `10 s`：

```bash
python3 chassis_test_common.py \
  --waveform step \
  --linear 0.01 \
  --angular 0 \
  --duration 10
```

测试阶跃角速度时，将 `--linear` 设置为 `0`，再通过 `--angular` 设置目标角速度。

### 梯形速度

以下命令以 `0.1 m/s²` 加速到 `0.2 m/s`，保持 `3 s`，再以相同加速度减速到零：

```bash
python3 chassis_test_common.py \
  --waveform trapezoid \
  --linear 0.2 \
  --angular 0 \
  --linear-accel 0.1 \
  --linear-hold 3
```

测试梯形角速度时使用 `--angular`、`--angular-accel` 和 `--angular-hold`，并将 `--linear` 设置为 `0`。

### 正弦速度

以下命令让角速度在 `-0.3～0.3 rad/s` 之间变化，频率为 `0.2 Hz`，持续 `10 s`：

```bash
python3 chassis_test_common.py \
  --waveform sine \
  --linear 0 \
  --angular 0.3 \
  --sine-frequency 0.2 \
  --duration 10
```

## 直线底盘直控测试

单次可视化测试，向前运动 `0.5 m`：

```bash
python3 precise_straight_action_test.py \
  --distance-m 0.5 \
  --mode visual
```

压测 `100` 次：

```bash
python3 precise_straight_action_test.py \
  --distance-m 0.5 \
  --mode stress \
  --test-count 100
```

压测默认按照 `+0.5 m、-0.5 m、+0.5 m……` 前后交替执行。`--test-interval` 表示每轮停车观察完成后的额外等待时间，默认 `1.0 s`。只有确认直线空间足够时，才使用 `--same-direction` 持续同向测试。

## 原地转向底盘直控测试

单次可视化测试，逆时针转动 `90°`：

```bash
python3 precise_rotation_action_test.py \
  --angle-deg 90 \
  --mode visual
```

压测 `100` 次：

```bash
python3 precise_rotation_action_test.py \
  --angle-deg 180 \
  --mode stress \
  --test-count 100
```

压测默认按照 `+180°、-180°、+180°……` 交替执行。只有确认线束和周围空间允许持续同向旋转时，才使用 `--same-direction`。

## 测试模式和结果

| 模式 | 用途 | 输出内容 |
| --- | --- | --- |
| `visual` | 单次测试和观察曲线 | CSV、JSON、TXT、SVG和PNG |
| `stress` | 多次重复测试 | 终端逐轮结果和`stress_report.txt` |

结果默认保存在：

```text
dexe_mobile_nav/mobile_perception/test/results/
```

可视化模式的主要文件：

- `data.csv`：逐采样原始数据。
- `summary.json`：便于程序读取的统计结果。
- `report.txt`：便于直接查看的中文结果。
- `report.svg` / `report.png`：速度、位移或角度曲线。

每次运行都会创建独立的时间戳目录，不会覆盖以前的结果。如果当前环境无法生成 PNG，脚本会保留 SVG 和其他报告。

## 常用选项

| 参数 | 说明 |
| --- | --- |
| `--mode visual` | 单次测试并生成曲线。 |
| `--mode stress` | 多次压测，不生成每轮曲线。 |
| `--test-count 100` | 设置压测次数。 |
| `--test-interval 1.0` | 设置压测轮次之间的额外等待时间，单位为秒。 |
| `--same-direction` | 压测始终同向执行，使用前必须确认空间安全。 |
| `--control-reference-frame odom` | 设置直线和转向测试的误差参考系，应与控制节点配置一致。 |
| `--output-root <目录>` | 修改测试结果保存位置。 |
| `--report-format svg|png|both` | 设置可视化报告格式。 |

查看某个脚本的全部参数和默认值：

```bash
python3 chassis_test_common.py --help
python3 precise_straight_action_test.py --help
python3 precise_rotation_action_test.py --help
```

## 注意事项

- 速度响应脚本使用 `/feedback/odom`；直线和转向脚本还使用 `odom -> base_link` TF，参考系应与底盘直控节点保持一致。
- `/control/cmd_vel` 存在多个发布者时，曲线可能混合多个节点的消息。测试前应停止其他控制任务。
- Action 返回成功表示控制节点满足自身到位条件，不代表报告中的最终误差一定为零。
- 修改 `dexe_mobile_nav/mobile_perception/config/mobile_control_action_params.yaml` 后需要重启底盘直控节点，再重新测试。
