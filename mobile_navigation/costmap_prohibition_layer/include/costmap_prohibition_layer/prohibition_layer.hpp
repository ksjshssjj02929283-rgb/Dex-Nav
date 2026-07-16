// Copyright 2026 DEXE Robot Engineer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// 核心多边形栅格化算法（rasterizePolygon/polygonOutlineCells/raytrace/
// setPolygonCost）移植自 rst-tu-dortmund/costmap_prohibition_layer（ROS1，
// BSD License；原作者 2016 年 Stephan Kurzawe，TU Dortmund ICS Engineering）。
// 数据源从 ROS1 版的 YAML 参数改为订阅 map_asset_manager 的
// nav_interfaces/AreaInfoArray（RESTRICTED_AREA + VIRTUAL_WALL），详见
// 17 号方案文档 §三。

#ifndef COSTMAP_PROHIBITION_LAYER__PROHIBITION_LAYER_HPP_
#define COSTMAP_PROHIBITION_LAYER__PROHIBITION_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_interfaces/msg/area_info_array.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace costmap_prohibition_layer
{

// 整数格坐标点（对应 map 坐标系，可越界为负——raytrace 需要）。
struct PointInt
{
  int x;
  int y;
};

// 用 Area/Room 语义资产（RESTRICTED_AREA 禁行区 + VIRTUAL_WALL 虚拟墙）直接
// 栅格化到 costmap，替代"生成全图 mask + Nav2 KeepoutFilter 消费"的方案
// （旧方案见 navigation_adapter::AreaFilterAdapter 的 keepout 部分）。
//
// 与旧方案的核心区别：本层只在内存里存多边形顶点（几十到几百个点，几乎零
// 开销），updateBounds()/updateCosts() 只处理这些多边形自身的包围盒，不需要
// 分配、发布、订阅一张与地图同分辨率的 mask。
class ProhibitionLayer : public nav2_costmap_2d::Layer
{
public:
  ProhibitionLayer() = default;
  ~ProhibitionLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  void onFootprintChanged() override {}
  bool isClearable() override {return false;}  // 语义禁行约束，不应被"清 costmap"恢复行为清掉

  // 暴露给单测：绕开 ROS 订阅，直接喂一批 Area 数据触发重建（等价于
  // incomingAreas() 收到消息）。
  void rebuildFromAreasForTest(const nav_interfaces::msg::AreaInfoArray & msg)
  {
    rebuildFromAreas(msg);
  }
  // 暴露给单测：绕开 onInitialize()（需要真实 LifecycleNode），直接设置
  // enabled_/fill_polygons_ 这两个决定 updateBounds/updateCosts 是否早退的开关。
  void setEnabledForTest(bool enabled, bool fill_polygons)
  {
    enabled_ = enabled;
    fill_polygons_ = fill_polygons;
  }

private:
  void incomingAreas(const nav_interfaces::msg::AreaInfoArray::SharedPtr msg);

  // 用最新一批 Area 数据重建 prohibition_polygons_ + 世界系包围盒。
  void rebuildFromAreas(const nav_interfaces::msg::AreaInfoArray & msg);
  void computeMapBounds();

  // 把一个多边形（世界坐标）栅格化写入 costmap，cost 统一为 LETHAL_OBSTACLE。
  void setPolygonCost(
    nav2_costmap_2d::Costmap2D & master_grid,
    const std::vector<geometry_msgs::msg::Point> & polygon,
    unsigned char cost,
    int min_i, int min_j, int max_i, int max_j,
    bool fill_polygon);

  rclcpp::Subscription<nav_interfaces::msg::AreaInfoArray>::SharedPtr areas_sub_;

  std::mutex data_mutex_;
  bool fill_polygons_{true};
  std::string areas_topic_;
  double virtual_wall_width_{0.10};

  // 世界坐标系下的多边形集合：RESTRICTED_AREA 直接是原始多边形；VIRTUAL_WALL
  // 的每一段折线都转换成一个"沿法向偏移 virtual_wall_width_ 得到的窄四边形"
  // （与 ROS1 参考实现处理"线"输入的手法一致），这样禁行区和虚拟墙可以复用
  // 同一套多边形栅格化代码，不需要为墙单独写圆形膨胀逻辑。
  std::vector<std::vector<geometry_msgs::msg::Point>> prohibition_polygons_;

  double min_x_{0.0};
  double min_y_{0.0};
  double max_x_{0.0};
  double max_y_{0.0};

  // ── 删除/缩小多边形的 dirty-region 补偿 ──────────────────────────
  // LayeredCostmap::updateMap() 只在"本轮 updateBounds() 并集"范围内
  // resetMap()+repaint；本层 updateBounds() 若只报告"当前多边形集合"的
  // 包围盒，一个多边形被删除/缩小后，它原来所在的区域再也不会被任何一轮
  // 包含进去，之前打上的 LETHAL cost 会永久残留（除非恰好被别的 layer
  // 的范围覆盖到）。修复：每次 rebuildFromAreas() 后，下一次 updateBounds()
  // 额外并入"上一次实际上报出去的范围"一次（把旧区域也纳入这轮的
  // reset+repaint），之后收敛回只反映当前范围。
  double last_reported_min_x_{0.0};
  double last_reported_min_y_{0.0};
  double last_reported_max_x_{0.0};
  double last_reported_max_y_{0.0};
  bool need_bounds_clear_pass_{false};
};

// ── 纯几何函数（无 ROS/costmap 依赖，单测直接调用；均移植自参考实现）──

// 提取多边形轮廓经过的格子（Bresenham 逐边 raytrace，含首尾闭合边）。
void polygonOutlineCells(
  const std::vector<PointInt> & polygon,
  std::vector<PointInt> & polygon_cells);

// 两点间 Bresenham 直线光栅化，允许越界坐标（不依赖 costmap 尺寸）。
void raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt> & cells);

// 多边形栅格化：先取轮廓格子，fill=true 时再用扫描线法填充内部。
// 多边形至少 3 个点，否则不产出任何格子。
void rasterizePolygon(
  const std::vector<PointInt> & polygon,
  std::vector<PointInt> & polygon_cells,
  bool fill);

}  // namespace costmap_prohibition_layer

#endif  // COSTMAP_PROHIBITION_LAYER__PROHIBITION_LAYER_HPP_
