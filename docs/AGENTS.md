# docs 目录协作规则

## 维护范围

- 本目录用于记录仓库级文档和维护约定。
- 修改仓库级元数据、运行方式、配置策略或跨包规则时，应同步更新本目录文档。
- 本目录当前还维护 `dexe_sensor_process`、`mobile_perception`、`mobile_navigation` 三个核心目录的功能总览。
- 修改上述目录的节点、launch、参数、topic、action、service、TF frame、costmap observation source 或导航主链路时，应同步更新 `docs/README.md` 中对应功能说明。

## 功能总览维护约定

- `dexe_sensor_process` 说明应覆盖 RGBD 输入、点云输出、滤波参数、launch 入口，以及与 Nav2 costmap observation source 的关系。
- `mobile_perception` 说明应覆盖 AprilTag 检测、tag 对齐 TF、精控节点、精控 action、障碍检测、充电位姿采集和 Python 辅助脚本边界。
- `mobile_navigation` 说明应覆盖 Nav2 bringup、导航生命周期节点、BT 规划控制链路、planner/controller/costmap 关键插件和 W1 参数文件。
- 如果某个参数已经声明但源码尚未实际使用，应在文档中明确“保留参数”或“当前未实际应用”，避免后续误判运行行为。
- 记录功能时优先写实际入口文件和实际话题，不写脱离当前仓库的 Nav2 通用说明。

## package.xml 维护约定

- 根目录 `package.xml` 的 `version` 作为仓库级版本来源。
- 功能包 `package.xml` 应同步仓库级版本号和 maintainer，当前 maintainer 为 `dexforce <dexforce@dexforce.com>`。
- 功能包 `description` 保持包自身说明，不复用根包的 CI 描述。
- Dexforce 自研包使用 `Proprietary` license。
- `mobile_navigation` 下的 Nav2 上游包保留原有 Apache、BSD、MIT、LGPL 等 license，不追加 `Proprietary`，不要删除上游 license 声明。

## 验证约束

- 默认不执行 `colcon build` 或其他构建命令。
- 修改 `package.xml` 后优先使用 XML 解析、`rg` 文本检查等非构建方式验证。

## Nav2 行为树文档维护约束

- 修改 `mobile_navigation/nav2_bt_navigator`、`mobile_navigation/nav2_behavior_tree`、`mobile_navigation/nav2_planner`、`mobile_navigation/nav2_controller` 中与导航主链路相关的行为时，应同步更新 `docs/README.md`。
- 涉及 `ComputePathToPose`、`FollowPath`、BT XML、planner/controller action 名称、blackboard key、controller/plugin ID 或重规划策略时，应说明数据如何在 `{goal}`、`{path}`、`compute_path_to_pose` 和 `follow_path` 之间传递。
- 只做代码阅读或知识梳理但结论会影响后续维护理解时，也应把稳定结论补充到 `docs/README.md`；不需要执行构建验证。
- 本仓库 `planner_server` 支持 `active_planner` 与 `override_planner_id`，说明规划链路时不要只按 Nav2 上游默认 `planner_id` 语义解释。

## nav2_theta_star_planner 维护约束

- 修改 `ThetaStar::isSafe()` 时，需要同时关注普通邻居扩展和 LOS 检查两个重载版本，避免两处安全判定不一致。
- 当前 Theta* 规划约束为只允许原始 costmap 自由栅格通行；膨胀层代价栅格应视为不可通行。
