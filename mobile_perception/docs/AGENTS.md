# mobile_perception/docs 维护规则

## 文档范围

- 本目录用于记录 `mobile_perception` 包的补充说明、变更背景和维护注意事项。
- 使用者入口仍以 `../README.md` 为准，维护者约束仍以 `../AGENTS.md` 为准。

## 更新要求

- 修改 `mobile_perception` 的 C++ 节点、Python 脚本、话题、服务、参数、launch 或配置时，同步检查本目录是否需要补充说明。
- 新增节点时，至少记录节点用途、主要接口、关键参数和运行方式。
- 文档默认使用中文；命令、路径、包名、节点名、topic、service 和参数名可保留英文。

## 当前补充记录

- `mobile_control_node` 的取消状态区分 `CANCEL CONTROL`（取消中）和 `CANCELED`（取消完成反馈），修改状态字符串前需要确认上游调用方。
- `mobile_control_action_node` 提供 `control/precise_control` action，直接发布 `control/cmd_vel`，不经过 `control/ctrl_pose`。
- `obstacle_detector_node` 已从旧工作区迁移到当前 `mobile_perception` 包，负责基于局部代价地图检测车体前方矩形区域障碍物。
