# NodeMap RViz Editor

该插件位于 `map_manager` 包内，但数据处理和交互代码全部放在独立
`nodemap_rviz_plugin/` 目录。它复用 `nav2_map_server` 已有的 NodeMap YAML
格式，不修改地图服务、Theta* 或 `map_manager` 运行节点。

## 使用流程

1. 编译并 source 工作空间后启动 RViz。
2. `Panels -> Add New Panel -> nodemap_rviz_plugin/NodeMapEditorPanel`。
3. 点击“导入地图”，选择原始 `map.yaml`。
4. 分别设置 L1/L2 的节点间距、最小净空、默认连接半径和起铺偏移。
5. 点击“生成三层节点图”。
6. 用下拉框选择 L1、L2 或 L3。L3 为原始自由栅格，只读。
7. “拖动节点 [M]”：按住节点拖动，释放后自动检查目标位置，并重算默认安全边。
8. “添加/删除连接 [E]”：依次点击两个节点。已有连接会删除，无连接会添加。
9. 点击“保存供 Theta* 使用”。

面板会自动建立两个 RViz Display：

- `NodeMap Editor Source Map`：`/nodemap_editor/map`
- `NodeMap Editor Graph`：`/nodemap_editor/markers`

颜色：

- 蓝色：L1
- 绿色：L2
- 橙色：L3
- 黄色边：人工添加的安全连接
- 红色边：关闭安全校验后强制添加、且穿越障碍物或不满足净空的连接

## 输出文件

默认保存到原始地图目录下的 `preprocessed/`：

```text
distance_map.yaml
distance_map.pgm
nodemap_global.yaml
nodemap_local.yaml
nodemap_raw.yaml
nodemap_raw_mask.pgm
```

Theta* 配置：

```yaml
ThetaStarBased:
  use_node_map: true
  node_map_yaml: "/absolute/path/preprocessed/nodemap_global.yaml"
```

`local_map_yaml`、`raw_map_yaml` 和 `distance_map_yaml` 留空时，现有 Theta*
会从同一目录自动推导。

## 当前 Theta* 分层状态

本插件不会修改规划器。当前工程的
`nav2_theta_star_planner/multi_layer_planner.hpp` 中
`kUseLocalLayer=false`，并且当前加载流程实际采用 **L1 + costmap 最后一段**；
因此现在启用 `use_node_map` 后会立即使用这里编辑的 L1，L2/L3 文件会按既有格式
完整生成和保存，但在打开规划器的 L2/L3 流程前不会参与实际路径搜索。
