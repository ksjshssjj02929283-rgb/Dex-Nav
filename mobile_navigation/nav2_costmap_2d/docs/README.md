# nav2_costmap_2d 使用文档

## ESDF Cost Layer

`nav2_costmap_2d::EsdfLayer` 是全局代价地图插件，用于从 master costmap 中已有的 `LETHAL_OBSTACLE` 栅格计算 2D 欧氏距离场，并把距离映射为连续代价。

该层的目标不是替换 Smac 或 Theta Star 的全局规划算法，而是让现有规划器通过 costmap 中的连续 clearance cost 更倾向于远离障碍物边缘。

## 推荐插件顺序

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "esdf_layer"]
      esdf_layer:
        plugin: "nav2_costmap_2d::EsdfLayer"
        enabled: true
        influence_radius: 2.0
        cost_scaling_factor: 3.0
        use_exponential_decay: true
        treat_unknown_as_obstacle: false
        combination_method: 1
```

`esdf_layer` 应放在会写入 lethal obstacle 的层之后。默认只把 `LETHAL_OBSTACLE` 当作距离场源点，不把普通 inflation 中间值或 ESDF 自身输出的中间 cost 当作障碍源。

## 参数说明

- `enabled`：是否启用 ESDF layer，默认 `true`。
- `influence_radius`：ESDF 代价影响半径，单位米，默认 `2.0`。
- `cost_scaling_factor`：指数衰减系数，默认 `3.0`。
- `use_exponential_decay`：为 `true` 时使用指数衰减，为 `false` 时使用线性衰减。
- `treat_unknown_as_obstacle`：是否把 `NO_INFORMATION` 作为距离场源点，默认 `false`。
- `combination_method`：与 master costmap 的合并方式，`0=Overwrite`、`1=Max`、`2=Addition`，默认 `1`。

## 调参建议

Smac Lattice / Hybrid 可配合增大 `cost_penalty`，Smac 2D 可调 `cost_travel_multiplier`，Theta Star 可调 `w_traversal_cost`。如果路径仍贴近障碍，优先增大 `influence_radius` 或规划器 cost 权重；如果路径绕行过大，降低 `cost_scaling_factor` 或规划器 cost 权重。
