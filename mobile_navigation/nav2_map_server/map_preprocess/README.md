# Map preprocess (3-layer distance + node maps)

Offline tool: build **distance map** and **3-layer node graphs** from `map.yaml` + `map.pgm`.

## Layers

| Layer | File | Purpose |
|-------|------|---------|
| L1 global | `nodemap_global.yaml` | Sparse graph, global Theta* (safe clearance) |
| L2 local | `nodemap_local.yaml` | Finer graph, approach refinement |
| L3 raw | `nodemap_raw.yaml` + `nodemap_raw_mask.pgm` | Original free grid (1 cell), final goal accuracy |

## Planning flow (Theta* plugin)

The current runtime uses the **L1 global graph plus a costmap final leg**. L2 and
L3 are still generated and saved in the common NodeMap format for visualization,
editing, and future refinement, but are not part of the active search while
`MultiLayerNodePlanner::kUseLocalLayer` remains `false`.

Global costmap still uses original map + inflation for local obstacle avoidance.

## Output layout

```
<map_dir>/
  map.yaml
  map.pgm
  preprocessed/
    distance_map.yaml
    distance_map.pgm
    nodemap_global.yaml
    nodemap_local.yaml
    nodemap_raw.yaml
    nodemap_raw_mask.pgm
```

## Build & preprocess

```bash
colcon build --packages-select nav2_map_server nav2_theta_star_planner \
  --allow-overriding nav2_map_server nav2_theta_star_planner
source install/setup.bash

ros2 run nav2_map_server map_preprocess \
  --map-yaml ~/.dexe/data/maps/<map_name>/map.yaml
```

## Visualize

```bash
python3 $(ros2 pkg prefix nav2_map_server)/share/nav2_map_server/map_preprocess/visualize_preprocessed_map.py \
  --map-yaml ~/.dexe/data/maps/<map_name>/map.yaml
```

Opens **two windows** (unless `--save` or `--no-interactive`):

1. **Dashboard** — original 2×2 panels (occupancy, distance field, node layers)
2. **Interactive nodemap** — hover any node to see grid-path and euclidean clearance vs `min_clearance`

```bash
# save dashboard PNG only
python3 .../visualize_preprocessed_map.py \
  --map-yaml ~/.dexe/data/maps/<map_name>/map.yaml \
  --save /tmp/map_viz.png

# interactive L1 + L2 together
python3 .../visualize_preprocessed_map.py --map-yaml .../map.yaml --interactive-layer all

# hide layers (dashboard only)
python3 .../visualize_preprocessed_map.py --map-yaml .../map.yaml --hide-raw
python3 .../visualize_preprocessed_map.py --map-yaml .../map.yaml --hide-global --hide-local
```

### RViz — 分层显示 node map

本次迁移不改 `nav2_bringup`。可单独启动 NodeMap visualizer，再在 RViz 中添加
`MarkerArray` 显示 `/nodemap_markers`：

```bash
ros2 launch nav2_map_server nodemap_visualizer.launch.py \
  map_yaml:=~/.dexe/data/maps/<map_name>/map.yaml
```

`map_yaml` 指向 **原始 `map.yaml`**（同目录需有 `preprocessed/`）。发布话题为
`/nodemap_markers`。

可选：`show_global` / `show_local` / `show_raw` / `show_nodemap:=false`

## Planner params (`ThetaStarBased`)

```yaml
use_node_map: true
node_map_yaml: "~/.dexe/data/maps/<map>/preprocessed/nodemap_global.yaml"
# local_map_yaml / raw_map_yaml default to same preprocessed/ folder
```

## Tuning flags (`map_preprocess`)

| Flag | Default | Layer |
|------|---------|-------|
| `--global-clearance` | 0.45 m | L1 |
| `--global-spacing` | 0.80 m | L1 |
| `--local-clearance` | 0.30 m | L2 |
| `--local-spacing` | 0.40 m | L2 |

L3 uses original free cells (spacing = map resolution, clearance = 0).

**Node placement:** L1/L2 use a regular grid (`step/2 + k·step` cells). Cells below `min_clearance` (8-connected distance field) are skipped. Edges connect within `connection_radius` if line-of-sight clearance is OK.
