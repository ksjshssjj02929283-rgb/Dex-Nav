# RPP / PID 离线调试工具

这个目录提供一个不连接实车、不修改生产 YAML 的调参实验台。工具直接读取当前 Nav2 YAML，使用二维差速运动学近似复现本仓库 RPP 的主要计算顺序，并把每层速度限制分别画出来。

## 能做什么

- 读取 `w1_nav2_params_diff.yaml` 中的 RPP、`SCurveSpeedPlanner`、`velocity_smoother` 和局部代价地图参数。
- 自动检查无效缩放区间、原地转向滞回、膨胀系数不匹配、RPP 与平滑器能力不一致等问题。
- 模拟主前瞻目标点、固定近场目标点、纯追踪曲率和路径原地转向状态。
- 分别显示规划速度、曲率限制、障碍限制、横向误差限制、朝向限制、角速度能力限制。
- 模拟速度 PID、速度平滑器和带响应滞后的差速底盘。
- 提供直线、1 m 弯、90°折角、S弯、高频小波动和动态障碍场景。
- 批量生成 PNG 和 CSV 报告。

## 重要边界

这是调参方向验证工具，不是实车数字孪生：

- 速度规划部分保留当前代码的约束顺序，但用恒加速度前后向传播近似 S 曲线，不复刻全部加加速度内部状态。
- 代价地图使用简化圆形障碍和距离模型。
- 底盘使用一阶响应加差速运动学，没有轮胎打滑、电机死区、定位噪声和通信抖动。
- 碰撞预测与生产 RPP 思路一致，但机器人轮廓近似为圆。
- 工具不会修改生产 YAML；`--set` 和界面滑块只修改内存中的临时参数。

因此它适合排除参数关系错误、比较调参趋势和缩小实车测试范围，最终参数仍需低速实车验证。

## 运行环境

仅使用当前系统已有依赖：

```text
Python 3
NumPy
PyYAML
Matplotlib
```

从仓库根目录运行。

也可以使用目录内的快捷脚本，下面两种写法等价：

```bash
./debug_tool/run_debug_tool.sh --check
python3 -m debug_tool.run --check
```

## 参数检查

```bash
python3 -m debug_tool.run --check
```

当前生产 YAML 会报告已知问题，例如朝向缩放区间反向、横向缩放无渐变区、膨胀系数不匹配和角速度能力不一致。

## 查看标准场景

```bash
python3 -m debug_tool.run --list-scenarios
```

## 打开交互界面

```bash
python3 -m debug_tool.run
```

会打开两个 Matplotlib 窗口：

- `DEXE RPP/PID 参数`：场景选择、参数滑块和运行按钮。
- `DEXE RPP/PID 仿真结果`：路径、实际轨迹、各层速度、曲率、误差和角速度。

滑块变化后点击“运行 RPP”。“恢复 YAML 参数”会恢复到加载时的 YAML 数值，不写入文件。
点击“打开速度 PID 阶跃响应”会再打开 PID 控制与阶跃响应窗口；拖动 Kp、Ki、Kd、
积分限幅和最大加速度即可实时观察上升速度、超调和稳态误差。

## 无界面生成一张图

```bash
python3 -m debug_tool.run --no-gui --scenario sharp_90 --output /tmp/rpp_debug
```

## 临时覆盖参数

```bash
python3 -m debug_tool.run --no-gui --scenario curve_1m \
  --set heading_error_scaling_min_angle=0.10 \
  --set heading_error_scaling_max_angle=0.35 \
  --set rotate_to_path_entry_angle=0.45
```

覆盖值仅用于本次运行。

## 批量报告

```bash
python3 -m debug_tool.run --batch --output /tmp/rpp_batch
```

输出包括：

```text
/tmp/rpp_batch/straight.png
/tmp/rpp_batch/curve_1m.png
/tmp/rpp_batch/sharp_90.png
...
/tmp/rpp_batch/summary.csv
```

CSV 包含是否完成、最大横向误差、平均速度、最大角速度、原地转向次数和最小障碍余量等指标，表头也使用中文。

## 运行测试

```bash
python3 -m unittest discover -s debug_tool/tests -v
```

## 文件结构

```text
debug_tool/
├── app.py             Matplotlib 交互界面
├── config.py          YAML 读取和参数合法性检查
├── control.py         RPP 各层约束、速度 PID、平滑器
├── geometry.py        路径投影、前瞻点插值、曲率
├── pid_lab.py         速度 PID 阶跃实验
├── plotting.py        统一绘图
├── run.py             命令行入口
├── run_debug_tool.sh  快捷启动脚本
├── scenarios.py       标准路径和动态障碍
├── simulator.py       二维闭环仿真
├── speed_profile.py   滚动速度规划近似
└── tests/             自动测试
```
