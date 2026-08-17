# NodeMap 规划迁移清单

## 迁移信息

- 来源分支：`origin/feature/peng.hu/omni_control_optimization`
- 目标分支：`feature/peng.hu/node_map_planning`
- 共同基线：`release_0.5.x`（`1a5e0b23ff63bf7b6b21cfc95804bedd78a4d0d2`）
- 迁移方式：按功能文件迁移并针对当前分支接入，不合并来源分支历史
- 当前状态：工作区改动，尚未创建提交

本次只迁移 NodeMap 数据、预处理、Theta* 节点图规划、通用几何平滑后处理、
NodeMap 可视化/编辑和相关离线演示代码。未移动或修改 `release_0.5.x` 分支指针。

## 新增文件（35 个）

### `nav2_map_server`：NodeMap 数据、预处理和可视化（10 个）

- `mobile_navigation/nav2_map_server/include/nav2_map_server/map_preprocess.hpp`
- `mobile_navigation/nav2_map_server/include/nav2_map_server/node_map_io.hpp`
- `mobile_navigation/nav2_map_server/include/nav2_map_server/node_map_types.hpp`
- `mobile_navigation/nav2_map_server/launch/nodemap_visualizer.launch.py`
- `mobile_navigation/nav2_map_server/map_preprocess/README.md`
- `mobile_navigation/nav2_map_server/map_preprocess/visualize_preprocessed_map.py`
- `mobile_navigation/nav2_map_server/src/map_preprocess/main.cpp`
- `mobile_navigation/nav2_map_server/src/map_preprocess/map_preprocess.cpp`
- `mobile_navigation/nav2_map_server/src/map_preprocess/node_map_io.cpp`
- `mobile_navigation/nav2_map_server/src/map_preprocess/nodemap_visualizer.cpp`

### `map_manager`：独立 NodeMap RViz 编辑插件（10 个）

- `mobile_manager/map_manager/nodemap_rviz_plugin/README.md`
- `mobile_manager/map_manager/nodemap_rviz_plugin/plugin_description.xml`
- `mobile_manager/map_manager/nodemap_rviz_plugin/include/nodemap_rviz_plugin/edit_edge_tool.hpp`
- `mobile_manager/map_manager/nodemap_rviz_plugin/include/nodemap_rviz_plugin/editor_model.hpp`
- `mobile_manager/map_manager/nodemap_rviz_plugin/include/nodemap_rviz_plugin/move_node_tool.hpp`
- `mobile_manager/map_manager/nodemap_rviz_plugin/include/nodemap_rviz_plugin/nodemap_editor_panel.hpp`
- `mobile_manager/map_manager/nodemap_rviz_plugin/src/edit_edge_tool.cpp`
- `mobile_manager/map_manager/nodemap_rviz_plugin/src/editor_model.cpp`
- `mobile_manager/map_manager/nodemap_rviz_plugin/src/move_node_tool.cpp`
- `mobile_manager/map_manager/nodemap_rviz_plugin/src/nodemap_editor_panel.cpp`

### `nav2_theta_star_planner`：节点图规划和平滑后处理（15 个）

- `mobile_navigation/nav2_theta_star_planner/include/nav2_theta_star_planner/curve_path_smoother.hpp`
- `mobile_navigation/nav2_theta_star_planner/include/nav2_theta_star_planner/l1_spline_path.hpp`
- `mobile_navigation/nav2_theta_star_planner/include/nav2_theta_star_planner/multi_layer_planner.hpp`
- `mobile_navigation/nav2_theta_star_planner/include/nav2_theta_star_planner/node_theta_star.hpp`
- `mobile_navigation/nav2_theta_star_planner/include/nav2_theta_star_planner/omni_path_smoother.hpp`
- `mobile_navigation/nav2_theta_star_planner/src/curve_path_smoother.cpp`
- `mobile_navigation/nav2_theta_star_planner/src/l1_spline_path.cpp`
- `mobile_navigation/nav2_theta_star_planner/src/multi_layer_planner.cpp`
- `mobile_navigation/nav2_theta_star_planner/src/node_theta_star.cpp`
- `mobile_navigation/nav2_theta_star_planner/src/omni_path_smoother.cpp`
- `mobile_navigation/nav2_theta_star_planner/test/map_plan_demo.launch.py`
- `mobile_navigation/nav2_theta_star_planner/test/plan_demo.rviz`
- `mobile_navigation/nav2_theta_star_planner/test/plan_demo_params.yaml`
- `mobile_navigation/nav2_theta_star_planner/test/plan_from_rviz.py`
- `mobile_navigation/nav2_theta_star_planner/test/run_map_plan_demo.sh`

`omni_path_smoother` 保留原文件名，但实现只依赖路径几何和 costmap，不包含舵轮
转向模型或全向控制指令，可供当前差速底盘使用。

## 修改文件（8 个）

- `mobile_navigation/nav2_map_server/CMakeLists.txt`：编译、安装并导出预处理库和可视化节点
- `mobile_navigation/nav2_map_server/package.xml`：增加可视化消息依赖
- `mobile_navigation/nav2_theta_star_planner/CMakeLists.txt`：接入 NodeMap/平滑源码并安装离线演示文件
- `mobile_navigation/nav2_theta_star_planner/package.xml`：增加 NodeMap 和演示运行依赖
- `mobile_navigation/nav2_theta_star_planner/include/nav2_theta_star_planner/theta_star_planner.hpp`：声明 NodeMap 规划与通用平滑接口
- `mobile_navigation/nav2_theta_star_planner/src/theta_star_planner.cpp`：接入 NodeMap 规划、回退和可选平滑后处理
- `mobile_manager/map_manager/CMakeLists.txt`：以独立可选目标接入 RViz 编辑插件，不改 `map_manager_node`
- `mobile_manager/map_manager/package.xml`：增加 RViz 插件依赖和导出信息

## 明确未迁移内容

- `swerve_steering_model.hpp/.cpp`
- `orientation_reference` 跨包参数和消息传递链路
- MPPI、全向/舵轮控制、`mobile_joy_control`、转向角控制相关改动
- 来源分支对 `test_theta_star.cpp` 的姿态方向测试改动
- 来源分支生成的 `nav2_map_server/test/preprocessed/` 二进制/结果文件；可由
  `map_preprocess` 使用现有测试地图重新生成
- `mobile_navigation/nav2_bringup` 下的所有配置和启动文件
- 用户工作区原有的未跟踪目录 `debug_tool/`

因此没有新增 `w1_nav2_params_omni.yaml`，也没有修改
`mobile_navigation/nav2_bringup/params/w1_nav2_params_diff.yaml`。

## 与差速逻辑的共存方式

- `use_node_map` 默认值为 `false`。
- `smooth_path` 和 `smooth_do_curve_fit` 默认值为 `false`。
- 上述功能均关闭时，栅格 Theta* 继续使用迁移前的 `linearInterpolation` 路径流程。
- NodeMap 加载或规划失败时会回退到原栅格 Theta*。
- 离线演示参数单独启用 NodeMap 和平滑，不会覆盖正式差速配置。
- 当前 NodeMap 运行路径为 **L1 全局节点图 + costmap 最后一段**；L2/L3 会生成、
  保存并用于可视化/编辑，但 `kUseLocalLayer=false` 时不参与实际路径搜索。

## 验证结果

- `nav2_map_server` 与 `nav2_theta_star_planner` 完整编译通过。
- NodeMap RViz 插件四个源文件和 Qt MOC 独立编译、链接通过。
- 当前工作区缺少 `map_manager` 原有必需包 `nav_interfaces`，所以无法对整个
  `map_manager` 包做一体构建；该阻塞发生在插件配置前，与本次迁移无关。
- `map_preprocess` 使用现有 `testmap.yaml` 离线运行成功，并在 `/tmp` 生成：
  `distance_map.pgm/yaml`、`nodemap_global.yaml`、`nodemap_local.yaml`、
  `nodemap_raw.yaml`、`nodemap_raw_mask.pgm`。
- 功能性 GTest：`test_theta_star`、`test_map_io`、
  `test_costmap_filter_info_server` 共 3 项通过。
- 本次涉及包的 `cpplint`、`flake8`、`uncrustify` 共 6 项通过。
- Python 可视化、launch 和演示脚本通过语法检查。
- `git diff --check` 通过；未发现舵轮、转向模型、`orientation_reference`、
  MPPI 或 `mobile_cmd` 残留引用。
