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
- `mobile_control_action_node` 默认通过 `mobile_control_action.launch.py` 启动，并加载 `mobile_control_action_params.yaml`。
- 当前应用的二维码检测以 `src/apriltag_alignment_tf_node.cpp` 为准；该节点按
  `start_apriltag_detection` / `stop_apriltag_detection` 服务创建和释放 detector、图像订阅、
  TF listener/broadcaster 与发布器。
- `apriltag_alignment_tf_node` 默认发布 `apriltag_target_pose`，目标点以
  `apriltag_aligned_<id>` 姿态为基准，沿 X 轴偏移 `target_forward_offset`，用于后续发送到
  `control/precise_control`。
- 维护二维码精控链路时，应同步检查 `map`、`base_link`、`camera_link` 的 TF 连通性，以及
  `apriltag_target_pose.header.frame_id` 是否能转换到 `map`。
- Action 精控链路中不要同时运行 `mobile_control_node` 与 `mobile_control_action_node` 控制同一个
  `control/cmd_vel`。
- `obstacle_detector_node` 已从旧工作区迁移到当前 `mobile_perception` 包，负责基于局部代价地图检测车体前方矩形区域障碍物。
