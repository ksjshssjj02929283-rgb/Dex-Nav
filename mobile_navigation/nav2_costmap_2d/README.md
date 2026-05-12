# Nav2 Costmap_2d

The costmap_2d package is responsible for building a 2D costmap of the environment, consisting of several "layers" of data about the environment. It can be initialized via the map server or a local rolling window and updates the layers by taking observations from sensors. A plugin interface allows for the layers to be combined into the costmap and finally inflated via an inflation radius based on the robot footprint. The nav2 version of the costmap_2d package is mostly a direct ROS2 port of the ROS1 navigation stack version, with minimal noteable changes necessary due to support in ROS2. 

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-costmaps.html) for additional parameter descriptions for the costmap and its included plugins. The [tutorials](https://navigation.ros.org/tutorials/index.html) and [first-time setup guides](https://navigation.ros.org/setup_guides/index.html) also provide helpful context for working with the costmap 2D package and its layers. A [tutorial](https://navigation.ros.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html) is also provided to explain how to create costmap plugins.

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available planner plugins. 

## ESDF Cost Layer

`nav2_costmap_2d::EsdfLayer` builds a continuous Euclidean-distance cost field from
`LETHAL_OBSTACLE` cells already present in the master costmap. It is intended for the
global costmap so planners such as Smac and Theta Star can prefer paths with larger
obstacle clearance through their existing cost-aware traversal terms.

Example global costmap plugin configuration:

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

`combination_method` follows the existing costmap layer convention:
`0=Overwrite`, `1=Max`, `2=Addition`. The recommended default is `Max` so the ESDF
layer does not lower obstacle or static-map costs. The layer only uses lethal cells
as ESDF source sites by default; intermediate inflation or ESDF costs are not treated
as obstacles.

## To visualize the voxels in RVIZ:
- Make sure `publish_voxel_map` in `voxel_layer` param's scope is set to `True`.
- Open a new terminal and run:
  ```ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid visualization_marker:=/my_marker```
    Here you can change `my_marker` to any topic name you like for the markers to be published on.

- Then add `my_marker` to RVIZ using the GUI.


### Errata:
- To see the markers in 3D, you will need to change the _view_ in RVIZ to a 3 dimensional view (e.g. orbit) from the RVIZ GUI.
- Currently due to some bug in rviz, you need to set the `fixed_frame` in the rviz display, to `odom` frame.
- Using pointcloud data from a saved bag file while using gazebo simulation can be troublesome due to the clock time skipping to an earlier time.

## Costmap Filters

### Overview

Costmap Filters - is a costmap layer-based instrument which provides an ability to apply to map spatial-dependent raster features named as filter-masks. These features are used in plugin algorithms when filling costmaps in order to allow robots to change their trajectory, behavior or speed when a robot enters/leaves an area marked in a filter masks. Examples of costmap filters include keep-out/safety zones where robots will never enter, speed restriction areas, preferred lanes for robots moving in industries and warehouses. More information about design, architecture of the feature and how it works could be found on Nav2 website: https://navigation.ros.org.
