# Map preprocess (3-layer distance + node maps)

Offline tool: build **distance map** and **3-layer node graphs** from `map.yaml` + `map.pgm`.

## Layers

| Layer | File | Purpose |
|-------|------|---------|
| L1 global | `nodemap_global.yaml` | Sparse graph, global Theta* (safe clearance) |
| L2 local | `nodemap_local.yaml` | Finer graph, approach refinement |
| L3 raw | `nodemap_raw.yaml` + `nodemap_raw_mask.pgm` | Original free grid (1 cell), final goal accuracy |

## Planning flow (Theta* plugin)

1. True goal → snap to **L1** sparse node → plan start → that node  
2. If goal is in obstacle on **L3 raw** layer → path ends at L1 node (no fine approach)  
3. Else: **L2** refines L1 endpoint → nearest local node near goal  
4. **L3** refines L2 endpoint → exact goal pose on original free cells  

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

已集成到 `nav2_bringup/launch/rviz_launch.py`（与 RViz 同启）：

```bash
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=true \
  rviz_config:=/path/to/omni_display.rviz \
  map_yaml:=~/.dexe/data/maps/<map_name>/map.yaml
```

`map_yaml` 指向 **原始 `map.yaml`**（同目录需有 `preprocessed/`）。RViz 里已预置 **NodeMap L1/L2/L3** MarkerArray（话题 `/nodemap_markers`）。

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
