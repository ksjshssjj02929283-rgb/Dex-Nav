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

#ifndef ROBOT_STORYTELLING__ROUTE_ATTRIBUTE_TELLER_HPP_
#define ROBOT_STORYTELLING__ROUTE_ATTRIBUTE_TELLER_HPP_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "robot_storytelling/teller_base.hpp"
#include "nav_interfaces/msg/route_edge.hpp"
#include "nav_interfaces/srv/get_route_graph.hpp"

namespace robot_storytelling
{

// 路网属性 teller（2026-07-14 新建，落地 DEC-010 的 CURRENT_EDGE 事件）。
//
// 计算基础是 odom × RouteGraph 边的世界系最近边匹配——与 AreaAttributeTeller
// 的 odom × Area/Room 多边形匹配是两条不同的计算路径，因此拆成独立 teller
// （详见 15/16 号方案文档、record/decisions.md DEC-010）。
//
// 约束（DEC-010）：
//   - 只做"现在压在哪条边"（世界系真值），不订阅 plan/RoutePlan（那是"将要
//     经过哪段"，属于 L5 SpeedLimitBridge/RobotTrajectory 的职责）。
//   - 输出只供调度/监控消费，不进控制链路（不产 SpeedLimit、不接 controller）。
//
// RouteGraph 通过 GetRouteGraph service 加载（自己维护一份轻量轮询，不依赖
// nav2_route_planner 的 RouteGraphLoader——避免 mobile_manager 包反向依赖
// mobile_navigation 包）。
class RouteAttributeTeller : public TellerBase
{
public:
  void Init(rclcpp::Node * node) override;
  void Update(
    const TellerContext & ctx,
    std::vector<nav_interfaces::msg::Story> * stories) override;
  std::string Name() const override {return "RouteAttributeTeller";}

  // 暴露给单测：对显式给定的 edges 跑一次带滞后状态机的匹配，不需要真实
  // GetRouteGraph service。滞后状态（current_edge_id_/candidate 计数）保存在
  // 对象上，构造新对象即可重置——与项目里 L4/L6 既有 teller "纯函数 + 状态在对
  // 象上"的风格一致。
  std::optional<nav_interfaces::msg::Story> updateWithEdges(
    double rx, double ry,
    const std::vector<nav_interfaces::msg::RouteEdge> & edges);

private:
  void onTimer();
  void onGraphResponse(
    rclcpp::Client<nav_interfaces::srv::GetRouteGraph>::SharedFuture future);

  rclcpp::Node * node_{nullptr};
  double match_distance_threshold_{0.5};
  int switch_hysteresis_frames_{3};
  int num_samples_per_edge_{20};
  std::string route_graph_service_name_;

  rclcpp::Client<nav_interfaces::srv::GetRouteGraph>::SharedPtr route_graph_client_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  bool request_in_flight_{false};
  bool graph_loaded_once_{false};

  std::vector<nav_interfaces::msg::RouteEdge> edges_;

  // 滞后状态机：current 是已确认的当前边；candidate 是正在积累连续命中帧数的
  // 候选新边；candidate_streak_ 达到 switch_hysteresis_frames_ 才切换。
  std::string current_edge_id_;
  std::string candidate_edge_id_;
  int candidate_streak_{0};
};

// ── 纯几何函数（无 ROS 依赖，单测直接调用）──────────────────────────

// 广义 Bezier 求值（De Casteljau）。control_points.size()==2 时退化为直线
// 插值，因此同一份代码同时覆盖 LINE 和 BEZIER 两种 edge_type，无需分支。
geometry_msgs::msg::Point evaluateBezier(
  const std::vector<geometry_msgs::msg::Point> & control_points, double t);

// 把一条边采样为 num_samples+1 个点的折线（含端点）。control_points 少于 2
// 个视为无效边，返回空 vector。
std::vector<std::pair<double, double>> sampleEdge(
  const nav_interfaces::msg::RouteEdge & edge, int num_samples);

// 点到折线（线段序列）的最短距离。
double pointToPolylineDistance(
  double px, double py,
  const std::vector<std::pair<double, double>> & polyline);

// 在 edges 中找到离 (rx, ry) 最近、且在 distance_threshold 内的边的下标；
// 找不到（edges 为空或全部超出阈值）返回 std::nullopt。
std::optional<std::size_t> findNearestEdge(
  double rx, double ry,
  const std::vector<nav_interfaces::msg::RouteEdge> & edges,
  double distance_threshold,
  int num_samples_per_edge = 20);

}  // namespace robot_storytelling

#endif  // ROBOT_STORYTELLING__ROUTE_ATTRIBUTE_TELLER_HPP_
