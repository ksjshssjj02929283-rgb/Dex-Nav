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

#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "costmap_prohibition_layer/prohibition_layer.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav_interfaces/msg/area_info.hpp"
#include "nav_interfaces/msg/area_info_array.hpp"

using costmap_prohibition_layer::PointInt;
using costmap_prohibition_layer::polygonOutlineCells;
using costmap_prohibition_layer::raytrace;
using costmap_prohibition_layer::ProhibitionLayer;
using costmap_prohibition_layer::rasterizePolygon;

TEST(ProhibitionLayerGeometry, RaytraceHorizontalLine)
{
  std::vector<PointInt> cells;
  raytrace(0, 0, 5, 0, cells);
  ASSERT_EQ(cells.size(), 6u);  // 含端点，5 步 + 1
  EXPECT_EQ(cells.front().x, 0);
  EXPECT_EQ(cells.back().x, 5);
  for (const auto & c : cells) {
    EXPECT_EQ(c.y, 0);
  }
}

TEST(ProhibitionLayerGeometry, RaytraceHandlesNegativeCoordinates)
{
  std::vector<PointInt> cells;
  raytrace(-3, -3, 3, 3, cells);
  EXPECT_FALSE(cells.empty());
  EXPECT_EQ(cells.front().x, -3);
  EXPECT_EQ(cells.back().x, 3);
}

TEST(ProhibitionLayerGeometry, PolygonOutlineCellsClosesLoop)
{
  // 一个正方形（4 个角点），轮廓应包含 4 条边（含闭合边）。
  std::vector<PointInt> square = {{0, 0}, {4, 0}, {4, 4}, {0, 4}};
  std::vector<PointInt> cells;
  polygonOutlineCells(square, cells);
  // 起点 (0,0) 应该出现（第一条边的起点）；闭合边终点也应回到 (0,0) 附近。
  bool found_origin = false;
  for (const auto & c : cells) {
    if (c.x == 0 && c.y == 0) {
      found_origin = true;
    }
  }
  EXPECT_TRUE(found_origin);
  EXPECT_GT(cells.size(), square.size());  // 轮廓格子数应多于顶点数（含边上采样）
}

TEST(ProhibitionLayerGeometry, RasterizePolygonTooFewPointsYieldsEmpty)
{
  std::vector<PointInt> line = {{0, 0}, {1, 1}};  // 只有 2 个点，不构成多边形
  std::vector<PointInt> cells;
  rasterizePolygon(line, cells, true);
  EXPECT_TRUE(cells.empty());
}

TEST(ProhibitionLayerGeometry, RasterizePolygonFillMarksInterior)
{
  std::vector<PointInt> square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  std::vector<PointInt> cells;
  rasterizePolygon(square, cells, true);

  bool found_interior = false;
  for (const auto & c : cells) {
    if (c.x == 5 && c.y == 5) {
      found_interior = true;
      break;
    }
  }
  EXPECT_TRUE(found_interior) << "fill=true 应该把多边形内部格子也标出来";
}

TEST(ProhibitionLayerGeometry, RasterizePolygonNoFillSkipsInterior)
{
  std::vector<PointInt> square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  std::vector<PointInt> outline_only;
  rasterizePolygon(square, outline_only, false);

  bool found_interior = false;
  for (const auto & c : outline_only) {
    if (c.x == 5 && c.y == 5) {
      found_interior = true;
      break;
    }
  }
  EXPECT_FALSE(found_interior) << "fill=false 只应产出轮廓，不应包含纯内部格子";
}

// ── setPolygonCost 需要一个真实 Costmap2D，覆盖端到端栅格化+写 cost ─────

TEST(ProhibitionLayerCostmap, WorldPolygonMarksLethalOnRealCostmap)
{
  // 10x10 格，分辨率 1.0m，原点 (0,0) —— 格坐标与世界坐标数值上一致，便于断言。
  nav2_costmap_2d::Costmap2D costmap(10, 10, 1.0, 0.0, 0.0, 0);

  std::vector<geometry_msgs::msg::Point> polygon;
  auto pt = [](double x, double y) {
      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y;
      return p;
    };
  polygon = {pt(2.0, 2.0), pt(7.0, 2.0), pt(7.0, 7.0), pt(2.0, 7.0)};

  std::vector<costmap_prohibition_layer::PointInt> map_polygon;
  for (const auto & p : polygon) {
    costmap_prohibition_layer::PointInt loc;
    costmap.worldToMapNoBounds(p.x, p.y, loc.x, loc.y);
    map_polygon.push_back(loc);
  }
  std::vector<costmap_prohibition_layer::PointInt> cells;
  rasterizePolygon(map_polygon, cells, true);
  for (const auto & c : cells) {
    if (c.x >= 0 && c.x < 10 && c.y >= 0 && c.y < 10) {
      costmap.setCost(
        static_cast<unsigned int>(c.x), static_cast<unsigned int>(c.y),
        nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  // 多边形内部（约 4,4）应为 LETHAL；多边形外（约 0,0 / 9,9）应保持 0（FREE）。
  EXPECT_EQ(costmap.getCost(4, 4), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(costmap.getCost(0, 0), 0);
  EXPECT_EQ(costmap.getCost(9, 9), 0);
}

// ── 删除/缩小多边形的 dirty-region 补偿（Area 运行时增删的正确性）──────

namespace
{
nav_interfaces::msg::AreaInfo restrictedArea(
  double x0, double y0, double x1, double y1)
{
  nav_interfaces::msg::AreaInfo area;
  area.type = nav_interfaces::msg::AreaInfo::RESTRICTED_AREA;
  auto p2d = [](double x, double y) {
      geometry_msgs::msg::Pose2D pose;
      pose.x = x;
      pose.y = y;
      return pose;
    };
  area.polygon = {p2d(x0, y0), p2d(x1, y0), p2d(x1, y1), p2d(x0, y1)};
  return area;
}

// updateBounds() 的四个 double* 参数模拟 LayeredCostmap 每轮的累加器
// （初始为 [max, max, lowest, lowest]，多个 layer 依次扩展取并集）。
struct BoundsAccumulator
{
  double min_x{std::numeric_limits<double>::max()};
  double min_y{std::numeric_limits<double>::max()};
  double max_x{std::numeric_limits<double>::lowest()};
  double max_y{std::numeric_limits<double>::lowest()};
};
}  // namespace

TEST(ProhibitionLayerDirtyBounds, RemovedAreaStillReportedOnceForClearing)
{
  ProhibitionLayer layer;
  layer.setEnabledForTest(true, true);

  // 第一批：一个区域在 [0,0]-[2,2]。
  nav_interfaces::msg::AreaInfoArray msg1;
  msg1.areas = {restrictedArea(0.0, 0.0, 2.0, 2.0)};
  layer.rebuildFromAreasForTest(msg1);

  BoundsAccumulator acc1;
  layer.updateBounds(0, 0, 0, &acc1.min_x, &acc1.min_y, &acc1.max_x, &acc1.max_y);
  EXPECT_LE(acc1.min_x, 0.0);
  EXPECT_GE(acc1.max_x, 2.0);

  // 第二批：区域被删除（空列表）。
  nav_interfaces::msg::AreaInfoArray msg2;
  layer.rebuildFromAreasForTest(msg2);

  // 删除后的第一次 updateBounds()：即使当前多边形集合已空，也必须仍然
  // 报告旧区域 [0,0]-[2,2]，否则 LayeredCostmap 不会 reset 那片区域，
  // 之前打的 LETHAL 标记会永久残留。
  BoundsAccumulator acc2;
  layer.updateBounds(0, 0, 0, &acc2.min_x, &acc2.min_y, &acc2.max_x, &acc2.max_y);
  EXPECT_LE(acc2.min_x, 0.0) << "删除后第一轮应仍包含旧区域，否则残留的 LETHAL 标记永远不会被清除";
  EXPECT_GE(acc2.max_x, 2.0);
  EXPECT_LE(acc2.min_y, 0.0);
  EXPECT_GE(acc2.max_y, 2.0);

  // 第三次 updateBounds()（没有新的 rebuild）：应该收敛——不再包含旧区域
  // （多边形集合已空且清理已在上一轮完成，不应无限期重复报告旧范围）。
  BoundsAccumulator acc3;
  layer.updateBounds(0, 0, 0, &acc3.min_x, &acc3.min_y, &acc3.max_x, &acc3.max_y);
  EXPECT_GT(acc3.min_x, acc3.max_x) << "已收敛，不应再报告任何范围（累加器应保持初始的空状态）";
}

TEST(ProhibitionLayerDirtyBounds, MovedAreaReportsUnionOfOldAndNewOnce)
{
  ProhibitionLayer layer;
  layer.setEnabledForTest(true, true);

  // 第一批：区域在 [0,0]-[1,1]。
  nav_interfaces::msg::AreaInfoArray msg1;
  msg1.areas = {restrictedArea(0.0, 0.0, 1.0, 1.0)};
  layer.rebuildFromAreasForTest(msg1);
  BoundsAccumulator acc1;
  layer.updateBounds(0, 0, 0, &acc1.min_x, &acc1.min_y, &acc1.max_x, &acc1.max_y);

  // 第二批：同一个区域"搬家"到 [10,10]-[11,11]（旧区域应该被清掉）。
  nav_interfaces::msg::AreaInfoArray msg2;
  msg2.areas = {restrictedArea(10.0, 10.0, 11.0, 11.0)};
  layer.rebuildFromAreasForTest(msg2);

  BoundsAccumulator acc2;
  layer.updateBounds(0, 0, 0, &acc2.min_x, &acc2.min_y, &acc2.max_x, &acc2.max_y);
  // 应同时覆盖旧区域（约 0~1）和新区域（约 10~11）——两者的并集。
  EXPECT_LE(acc2.min_x, 0.0);
  EXPECT_GE(acc2.max_x, 11.0);

  // 下一轮收敛回只反映新区域。
  BoundsAccumulator acc3;
  layer.updateBounds(0, 0, 0, &acc3.min_x, &acc3.min_y, &acc3.max_x, &acc3.max_y);
  EXPECT_GE(acc3.min_x, 9.0) << "应收敛为只反映当前（新）区域，不再包含已清理过的旧区域";
  EXPECT_GE(acc3.max_x, 11.0);
}

TEST(ProhibitionLayerDirtyBounds, DisabledLayerNeverReportsBounds)
{
  ProhibitionLayer layer;
  layer.setEnabledForTest(false, true);

  nav_interfaces::msg::AreaInfoArray msg;
  msg.areas = {restrictedArea(0.0, 0.0, 2.0, 2.0)};
  layer.rebuildFromAreasForTest(msg);

  BoundsAccumulator acc;
  layer.updateBounds(0, 0, 0, &acc.min_x, &acc.min_y, &acc.max_x, &acc.max_y);
  EXPECT_GT(acc.min_x, acc.max_x) << "enabled=false 时不应报告任何范围";
}
