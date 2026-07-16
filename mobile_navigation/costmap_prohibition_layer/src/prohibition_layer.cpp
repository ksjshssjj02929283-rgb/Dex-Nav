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
// rasterizePolygon/polygonOutlineCells/raytrace/setPolygonCost 算法移植自
// rst-tu-dortmund/costmap_prohibition_layer（ROS1，BSD License；原作者
// 2016 年 Stephan Kurzawe，TU Dortmund ICS Engineering）。

#include "costmap_prohibition_layer/prohibition_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(costmap_prohibition_layer::ProhibitionLayer, nav2_costmap_2d::Layer)

namespace costmap_prohibition_layer
{

using nav2_costmap_2d::LETHAL_OBSTACLE;

// ── 纯几何函数（移植自参考实现，逻辑不变）───────────────────────────

void raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt> & cells)
{
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  PointInt pt{x0, y0};
  int n = 1 + dx + dy;
  const int x_inc = (x1 > x0) ? 1 : -1;
  const int y_inc = (y1 > y0) ? 1 : -1;
  int error = dx - dy;
  dx *= 2;
  dy *= 2;

  for (; n > 0; --n) {
    cells.push_back(pt);
    if (error > 0) {
      pt.x += x_inc;
      error -= dy;
    } else {
      pt.y += y_inc;
      error += dx;
    }
  }
}

void polygonOutlineCells(
  const std::vector<PointInt> & polygon,
  std::vector<PointInt> & polygon_cells)
{
  if (polygon.empty()) {
    return;
  }
  for (std::size_t i = 0; i + 1 < polygon.size(); ++i) {
    raytrace(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y, polygon_cells);
  }
  const std::size_t last = polygon.size() - 1;
  // 闭合多边形：从最后一个点连回第一个点。
  raytrace(polygon[last].x, polygon[last].y, polygon[0].x, polygon[0].y, polygon_cells);
}

void rasterizePolygon(
  const std::vector<PointInt> & polygon,
  std::vector<PointInt> & polygon_cells,
  bool fill)
{
  if (polygon.size() < 3U) {
    return;
  }

  polygonOutlineCells(polygon, polygon_cells);

  if (!fill) {
    return;
  }

  // 按 x 排序（简单冒泡，量级通常几十到几百点，够用）。
  std::size_t i = 0;
  while (i + 1 < polygon_cells.size()) {
    if (polygon_cells[i].x > polygon_cells[i + 1].x) {
      std::swap(polygon_cells[i], polygon_cells[i + 1]);
      if (i > 0) {
        --i;
      }
    } else {
      ++i;
    }
  }

  i = 0;
  const int min_x = polygon_cells.front().x;
  const int max_x = polygon_cells.back().x;

  for (int x = min_x; x <= max_x; ++x) {
    if (i + 1 >= polygon_cells.size()) {
      break;
    }

    PointInt min_pt;
    PointInt max_pt;
    if (polygon_cells[i].y < polygon_cells[i + 1].y) {
      min_pt = polygon_cells[i];
      max_pt = polygon_cells[i + 1];
    } else {
      min_pt = polygon_cells[i + 1];
      max_pt = polygon_cells[i];
    }
    i += 2;

    while (i < polygon_cells.size() && polygon_cells[i].x == x) {
      if (polygon_cells[i].y < min_pt.y) {
        min_pt = polygon_cells[i];
      } else if (polygon_cells[i].y > max_pt.y) {
        max_pt = polygon_cells[i];
      }
      ++i;
    }

    for (int y = min_pt.y; y < max_pt.y; ++y) {
      polygon_cells.push_back(PointInt{x, y});
    }
  }
}

namespace
{
// 把一段虚拟墙折线（两端点）转换成一个沿法向对称偏移 width 的窄四边形，
// 这样禁行区（原生多边形）和虚拟墙（线段）可以复用同一份 setPolygonCost。
std::vector<geometry_msgs::msg::Point> offsetWallSegment(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b,
  double width)
{
  double nx = b.y - a.y;
  double ny = a.x - b.x;
  const double len = std::hypot(nx, ny);
  if (len < 1e-9) {
    return {};
  }
  const double half = width * 0.5;
  nx = nx / len * half;
  ny = ny / len * half;

  geometry_msgs::msg::Point a1 = a;
  a1.x -= nx;
  a1.y -= ny;
  geometry_msgs::msg::Point b1 = b;
  b1.x -= nx;
  b1.y -= ny;
  geometry_msgs::msg::Point b2 = b;
  b2.x += nx;
  b2.y += ny;
  geometry_msgs::msg::Point a2 = a;
  a2.x += nx;
  a2.y += ny;
  return {a1, b1, b2, a2};
}
}  // namespace

// ── ProhibitionLayer ────────────────────────────────────────────────

void ProhibitionLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"ProhibitionLayer: failed to lock node"};
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("fill_polygons", rclcpp::ParameterValue(true));
  declareParameter("areas_topic", rclcpp::ParameterValue(std::string("/map_asset_manager/areas")));
  declareParameter("virtual_wall_width", rclcpp::ParameterValue(0.10));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "fill_polygons", fill_polygons_);
  node->get_parameter(name_ + "." + "areas_topic", areas_topic_);
  node->get_parameter(name_ + "." + "virtual_wall_width", virtual_wall_width_);

  current_ = true;
  min_x_ = min_y_ = max_x_ = max_y_ = 0.0;

  rclcpp::QoS latched(rclcpp::KeepLast(1));
  latched.transient_local().reliable();

  areas_sub_ = node->create_subscription<nav_interfaces::msg::AreaInfoArray>(
    areas_topic_, latched,
    [this](const nav_interfaces::msg::AreaInfoArray::SharedPtr msg) {incomingAreas(msg);});

  RCLCPP_INFO(
    logger_,
    "ProhibitionLayer initialized: areas_topic=%s fill_polygons=%s virtual_wall_width=%.2fm",
    areas_topic_.c_str(), fill_polygons_ ? "true" : "false", virtual_wall_width_);
}

void ProhibitionLayer::incomingAreas(const nav_interfaces::msg::AreaInfoArray::SharedPtr msg)
{
  rebuildFromAreas(*msg);
}

void ProhibitionLayer::rebuildFromAreas(const nav_interfaces::msg::AreaInfoArray & msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  prohibition_polygons_.clear();

  for (const auto & area : msg.areas) {
    if (area.type == nav_interfaces::msg::AreaInfo::RESTRICTED_AREA) {
      if (area.polygon.size() < 3U) {
        continue;
      }
      std::vector<geometry_msgs::msg::Point> poly;
      poly.reserve(area.polygon.size());
      for (const auto & p2d : area.polygon) {
        geometry_msgs::msg::Point p;
        p.x = p2d.x;
        p.y = p2d.y;
        poly.push_back(p);
      }
      prohibition_polygons_.push_back(std::move(poly));
    } else if (area.type == nav_interfaces::msg::AreaInfo::VIRTUAL_WALL) {
      if (area.polygon.size() < 2U) {
        continue;
      }
      for (std::size_t i = 0; i + 1 < area.polygon.size(); ++i) {
        geometry_msgs::msg::Point a;
        a.x = area.polygon[i].x;
        a.y = area.polygon[i].y;
        geometry_msgs::msg::Point b;
        b.x = area.polygon[i + 1].x;
        b.y = area.polygon[i + 1].y;
        auto quad = offsetWallSegment(a, b, virtual_wall_width_);
        if (!quad.empty()) {
          prohibition_polygons_.push_back(std::move(quad));
        }
      }
    }
  }

  computeMapBounds();
  need_bounds_clear_pass_ = true;
  current_ = true;
}

void ProhibitionLayer::computeMapBounds()
{
  // 调用方（rebuildFromAreas）已持有 data_mutex_，这里不再重复加锁。
  // 用 max()/lowest() 而非 0.0 做初始基线——0.0 会在多边形整体不覆盖世界
  // 原点时把包围盒错误地"钉"在 0 附近（例如多边形都在 (10,10) 附近，
  // min_x_ 却因为与 0.0 取 min 而永远是 0），导致 updateBounds() 的收敛
  // 逻辑失效。空多边形列表时 min_x_=max()/max_x_=lowest() 会让后续与
  // 累加器的 std::min/std::max 运算天然不产生任何扩张，语义上正确表示
  // "当前没有多边形贡献任何范围"。
  min_x_ = std::numeric_limits<double>::max();
  min_y_ = std::numeric_limits<double>::max();
  max_x_ = std::numeric_limits<double>::lowest();
  max_y_ = std::numeric_limits<double>::lowest();
  for (const auto & polygon : prohibition_polygons_) {
    for (const auto & p : polygon) {
      min_x_ = std::min(min_x_, p.x);
      min_y_ = std::min(min_y_, p.y);
      max_x_ = std::max(max_x_, p.x);
      max_y_ = std::max(max_y_, p.y);
    }
  }
}

void ProhibitionLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  if (prohibition_polygons_.empty() && !need_bounds_clear_pass_) {
    return;
  }

  double report_min_x = min_x_;
  double report_min_y = min_y_;
  double report_max_x = max_x_;
  double report_max_y = max_y_;

  if (need_bounds_clear_pass_) {
    // 并入上一次实际上报的范围（即"变化前"的旧区域），确保被删除/缩小的
    // 多边形所在区域这一轮也会被 LayeredCostmap reset+repaint；只补这一轮，
    // 补完立即消费掉这个标志，之后收敛回只反映当前范围。
    report_min_x = std::min(report_min_x, last_reported_min_x_);
    report_min_y = std::min(report_min_y, last_reported_min_y_);
    report_max_x = std::max(report_max_x, last_reported_max_x_);
    report_max_y = std::max(report_max_y, last_reported_max_y_);
    need_bounds_clear_pass_ = false;
  }

  *min_x = std::min(*min_x, report_min_x);
  *min_y = std::min(*min_y, report_min_y);
  *max_x = std::max(*max_x, report_max_x);
  *max_y = std::max(*max_y, report_max_y);

  last_reported_min_x_ = report_min_x;
  last_reported_min_y_ = report_min_y;
  last_reported_max_x_ = report_max_x;
  last_reported_max_y_ = report_max_y;
}

void ProhibitionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  for (const auto & polygon : prohibition_polygons_) {
    setPolygonCost(
      master_grid, polygon, LETHAL_OBSTACLE, min_i, min_j, max_i, max_j,
      fill_polygons_);
  }
}

void ProhibitionLayer::setPolygonCost(
  nav2_costmap_2d::Costmap2D & master_grid,
  const std::vector<geometry_msgs::msg::Point> & polygon,
  unsigned char cost,
  int min_i, int min_j, int max_i, int max_j,
  bool fill_polygon)
{
  std::vector<PointInt> map_polygon;
  map_polygon.reserve(polygon.size());
  for (const auto & p : polygon) {
    PointInt loc;
    master_grid.worldToMapNoBounds(p.x, p.y, loc.x, loc.y);
    map_polygon.push_back(loc);
  }

  std::vector<PointInt> polygon_cells;
  rasterizePolygon(map_polygon, polygon_cells, fill_polygon);

  for (const auto & cell : polygon_cells) {
    if (cell.x < min_i || cell.x >= max_i || cell.y < min_j || cell.y >= max_j) {
      continue;
    }
    master_grid.setCost(
      static_cast<unsigned int>(cell.x), static_cast<unsigned int>(cell.y), cost);
  }
}

void ProhibitionLayer::reset()
{
  current_ = true;
}

}  // namespace costmap_prohibition_layer
