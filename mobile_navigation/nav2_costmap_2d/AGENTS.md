# AGENTS.md

## 适用范围

本文件作用于 `mobile_navigation/nav2_costmap_2d` 目录。

## 包定位

- 该包是 Navigation2 的二维代价地图核心实现。
- 分析问题时，优先区分：
  - `plugins`：普通 layer
  - `filters`：基于 mask 的区域规则
- 重点入口文件：
  - `src/costmap_2d_ros.cpp`
  - `src/layered_costmap.cpp`
  - `costmap_plugins.xml`

## 阅读顺序建议

当任务涉及本包时，优先按下面顺序建立上下文：

1. `docs/README.md`
2. `costmap_plugins.xml`
3. `src/layered_costmap.cpp`
4. 对应插件实现文件

## 本包现状提示

- 当前包已经有 `INTRODUCT.md`，但更偏概览，后续优先维护 `docs/README.md` 作为主整理文档。
- 当前仓库配置里，`nav2_bringup/params/w1_nav2_rgbd_params.yaml` 已启用：
  - `keepout_filter`
  - `speed_filter`
- `KeepoutFilter` 的输入是 `OccupancyGrid mask`，不是“两点位虚拟墙”原生接口。

## 修改约束

- 默认不要做大范围重构。
- 优先保持 Nav2 原始结构和插件接口不变。
- 修改 `layered_costmap`、`costmap_2d_ros`、`ObstacleLayer`、`VoxelLayer` 这类运行时敏感模块时，要特别注意：
  - 更新顺序
  - 锁和线程安全
  - rolling window 行为
  - TF 依赖
  - filter 与 plugin 的执行边界

## 验证偏好

- 默认不要执行 `colcon build`。
- 优先使用：
  - 定向源码核对
  - 参数链路核对
  - 现有单元测试阅读
  - 小粒度命令验证

## 文档维护

- 若新增 filter / layer 说明，优先更新 `docs/README.md`。
- 若修改了本包能力边界，尤其是禁行、限速、虚拟墙相关能力，需要同步更新文档结论。
