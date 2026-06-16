# 重置地图原点设计方案

**版本**：v1.0  
**日期**：2026-05-09  
**关联决策**：D037

---

## 1. 背景与问题

机器人长期运行后，导航地图的语义原点（map frame 在物理空间中的绑定点）可能因以下原因需要重置：

- 建图时起点选取不合理（如走廊中段），导致导航坐标与建筑坐标系偏差
- 多地图管理时统一原点约定（如楼层入口）
- 硬件改造后机器人停放位置与历史原点偏移

重置原点的本质是：**对所有地图相关数据统一施加一个 2D 刚体变换** $T_{inv}$（旧坐标系 → 新坐标系）：

$$T_{new\_origin} = (x_0, y_0, \theta_0) \quad \text{（新原点在旧坐标系的位姿）}$$

$$T_{inv} = T_{new\_origin}^{-1}$$

$$P_{new} = T_{inv} \cdot P_{old}$$

涉及数据：
| 数据类别 | 负责组件 | 具体内容 |
|---------|---------|---------|
| 地图栅格 + 元数据 | cartographer_ros | `map.pbstream`、`map.png`、`map.yaml` |
| 导航点 | map_asset_manager | 点位 (x, y, yaw) |
| 区域 | map_asset_manager | 多边形顶点列表 |
| 路网节点 | map_asset_manager | Junction (x, y) |
| 路网导航点 | map_asset_manager | NavPoint in route graph (x, y, yaw) |

---

## 2. 架构决策

### 2.1 方案对比

| 方案        | 说明                                                                                       | 优点                                             | 缺点                                                   |
| --------- | ---------------------------------------------------------------------------------------- | ---------------------------------------------- | ---------------------------------------------------- |
| **A**     | 在 map_asset_manager 中直接链接 cartographer 库                                                 | 单一入口，接口简洁                                      | map_asset_manager 引入重型 cartographer 依赖，违反包职责边界；二进制膨胀 |
| **B**     | map_asset_manager 子进程调用 reset_map_origin_tool                                            | 无新依赖，工具已验证                                     | map_asset_manager 承担非本职工作；进程管理复杂；错误处理困难              |
| **C**     | cartographer_ros 新增 ROS2 服务，map_asset_manager 调用                                         | 接口干净                                           | 需修改 cartographer_ros；两者仍有编排耦合                        |
| **D（推荐）** | **command_board 编排全流程；cartographer 侧子进程调用工具；map_asset_manager 提供 `transform_assets` 服务** | 各包只负责本域数据；工具已验证无需重写；command_board 已有定位生命周期管理能力 | command_board 编排逻辑稍复杂                                |

### 2.2 推荐方案 D 详解

**核心原则**：
- **map_asset_manager** 只负责自己拥有的数据（点位、区域、路网），提供 `transform_assets` 服务
- **cartographer 文件**（pbstream/png/yaml）由已有的 `reset_map_origin_tool` CLI 处理，command_board 以子进程方式调用（定位已停止，离线安全）
- **command_board** 是唯一的编排者，拥有定位生命周期控制能力

**为什么不选 A/B**：
- A：map_asset_manager 是地图语义管理包，不应依赖 cartographer 算法库
- B：将 pbstream 处理推入 map_asset_manager 属于职责越界；子进程管理在 map_asset_manager 内更难维护

**为什么选子进程而非 ROS2 服务（方案 C）**：
- reset_map_origin_tool 已实现并经过测试，重写为 ROS2 服务需要同等工作量但收益有限
- 子进程调用发生在定位停止之后，无并发风险
- 后续若 cartographer_ros 提供标准服务，只需替换 command_board 中的子进程调用，map_asset_manager 接口不变

---

## 3. 服务接口定义

### 3.1 map_asset_manager — transform_assets

```
# nav_interfaces/srv/TransformAssets.srv
string  map_name  # 目标地图名；空字符串 = 当前加载中的地图（RobotPose 入口专用）
float64 x         # 新原点在旧坐标系中的 x (m)
float64 y         # 新原点在旧坐标系中的 y (m)
float64 yaw       # 新原点在旧坐标系中的旋转 (rad)
---
bool   success
string message
int32  transformed_points     # 变换的导航点数
int32  transformed_areas      # 变换的区域数（含多边形顶点）
int32  transformed_junctions  # 变换的路网节点数
```

服务名：`/map_asset_manager/transform_assets`

**行为分支**：

| map_name 值 | 行为 |
|------------|------|
| 空字符串 或 `== current_map_name_` | 直接操作内存 store → 持久化 → 重新发布 markers |
| 非空且 != current_map_name_ | 从文件加载 → 变换 → 写回文件（不触碰内存，不发布 markers）|

### 3.2 command_board — 面向外部调用者

**入口①：按指定位姿重置任意地图**（`reset_map_origin_by_xyyaw`）：

```
# nav_interfaces/srv/ResetMapOriginByXYYaw.srv
string  map_name  # 目标地图名（必填，可以是任意地图，不限于当前使用中的地图）
float64 x         # (m)
float64 y         # (m)
float64 yaw       # (rad)
---
bool    success
string  message
float64 applied_x
float64 applied_y
float64 applied_yaw
```

服务名：`~/reset_map_origin_by_xyyaw`

**入口②：按当前机器人位姿重置当前地图**（`reset_map_origin_by_robot_pose`）：

```
# std_srvs/srv/Trigger
# 服务名: ~/reset_map_origin_by_robot_pose
```

仅操作当前加载的地图，需要停/启定位。

---

## 4. 完整编排流程

### 4.1 入口①：按 XYYaw 重置任意地图（轻量流程）

```
外部调用 ~/reset_map_origin_by_xyyaw {map_name, x, y, yaw}
         │
         ├─ ① subprocess: reset_map_origin_tool
         │       --map_name <map_name> --new_x <x> --new_y <y> --new_yaw <yaw>
         │       → 修改 map.pbstream / map.png / map.yaml
         │
         └─ ② 调用 /map_asset_manager/transform_assets {map_name, x, y, yaw}
                 → 如果是非当前地图：从文件加载 → 变换 → 写回（不发布 markers）
                 → 如果是当前地图：操作内存 store → 持久化 → 重新发布 markers

不停定位，不重载地图，机器人可保持运行。
```

### 4.2 入口②：按当前机器人位姿重置当前地图（完整流程）

```
外部调用 ~/reset_map_origin_by_robot_pose (Trigger)
         │
         ├─ 采样 TF map→base_link，11 帧均值 → (x̄, ȳ, θ̄)
         │
         ├─ ① 停止导航（cancel all nav goals）
         │
         ├─ ② 停止定位
         │       → /localization_node/stop_localization
         │
         ├─ ③+④ 并行（缩短停机时间）：
         │       ③ subprocess: reset_map_origin_tool
         │           --map_name <current_map_name> ...
         │           → 修改 map.pbstream / map.png / map.yaml
         │       ④ /map_asset_manager/transform_assets {current_map_name, x̄, ȳ, θ̄}
         │           → 内存 store 变换 → 持久化 → 重新发布 markers
         │
         ├─ ⑤ 重载地图
         │       → map_manager/load_map {current_map_name}
         │
         └─ ⑥ 重启定位
                 → /localization_node/start_localization
                   （自动触发 relocalization）
```

---

## 5. 坐标变换数学

给定新原点在旧坐标系中的位姿 $(x_0, y_0, \theta_0)$：

$$R = \begin{pmatrix} \cos\theta_0 & \sin\theta_0 \\ -\sin\theta_0 & \cos\theta_0 \end{pmatrix}$$

2D 点变换：

$$\begin{pmatrix} x' \\ y' \end{pmatrix} = R \begin{pmatrix} x - x_0 \\ y - y_0 \end{pmatrix}$$

偏航角变换：

$$\theta' = \theta - \theta_0 \quad (\text{结果归一化到 } (-\pi, \pi])$$

---

## 6. 各模块变更清单

### nav_interfaces
- [x] 新增 `srv/TransformAssets.srv`（含 `map_name` 字段）
- [x] 新增 `srv/ResetMapOriginByXYYaw.srv`（含 `map_name` 字段）

### map_asset_manager
- [x] `CMakeLists.txt`：链接 TransformAssets.srv
- [x] `map_asset_manager_node.hpp`：声明 `transformAssetsCallback`、新服务成员
- [x] `map_asset_manager_node.cpp`：注册服务，实现双分支变换逻辑：
  - `map_name` 为空/等于当前地图 → 操作内存 store + 发布 markers
  - `map_name` 为其他非空值 → 从文件加载、变换、写回，不触碰内存

### command_board
- [x] 新增两条编排路径（by_robot_pose + by_xyyaw）
- [x] 机器人位姿采样逻辑（TF 订阅，11 帧均值）
- [x] 子进程调用 reset_map_origin_tool
- [x] **入口①（XYYaw）**：只做 tool + transform_assets，不停/启定位
- [x] **入口②（RobotPose）**：完整流程（停导航 → 停定位 → tool+assets 并行 → 重载地图 → 启定位）

---

## 7. 注意事项

1. **角度归一化**：变换后 yaw 需归一化到 $(-\pi, \pi]$，避免 nav2 偏航角越界
2. **XYYaw 入口无保护**：不停定位，调用方需自行确保变换期间机器人不在使用该地图定位（如离线地图操作场景），或接受定位漂移后由 relocalization 自动纠正
3. **多边形顶点顺序**：二维刚体变换不改变多边形的顺逆时针性，无需特殊处理
4. **幂等性**：多次调用 transform_assets 会累积变换。command_board 应保证只调用一次，或在调用前通过 reload_data 回退到原始状态
5. **pbstream 路径**：reset_map_origin_tool 的 `--map_name` 参数对应 `~/.dexe/data/maps/<name>`，需与 map_manager 的 maps_root 保持一致
