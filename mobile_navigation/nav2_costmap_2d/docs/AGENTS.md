# nav2_costmap_2d/docs 协作规则

## 文档范围

本目录记录 `nav2_costmap_2d` 中本地新增或调整的插件、参数和运行方式，尤其是与全局规划代价地图相关的行为。

## 维护要求

- 修改 costmap layer 插件、插件注册、参数名、默认值或合并语义时，同步更新 `README.md`。
- 说明 ESDF layer 时必须区分 `LETHAL_OBSTACLE` 源点、连续中间 cost 和 `NO_INFORMATION` 的处理方式。
- 文档示例应优先给出可直接放入 Nav2 参数文件的 YAML 片段。

## 验证约束

- 默认不主动执行 `colcon build`。
- 如果需要构建验证，先确认后优先执行 `colcon build --packages-select nav2_costmap_2d`。
