// Copyright (c) 2021, Samsung Research America
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
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

// added by hz
#include <queue>
#include <tuple>
#include <chrono>
// end

#include "Eigen/Core"
#include "nav2_smac_planner/smac_planner_lattice.hpp"

// #define BENCHMARK_TESTING

namespace nav2_smac_planner
{

using namespace std::chrono;  // NOLINT
using rcl_interfaces::msg::ParameterType;

SmacPlannerLattice::SmacPlannerLattice()
: _a_star(nullptr),
  _collision_checker(nullptr, 1, nullptr),
  _smoother(nullptr),
  _costmap(nullptr)
{
}

SmacPlannerLattice::~SmacPlannerLattice()
{
  RCLCPP_INFO(
    _logger, "Destroying plugin %s of type SmacPlannerLattice",
    _name.c_str());
}

void SmacPlannerLattice::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  _node = parent;
  auto node = parent.lock();
  _logger = node->get_logger();
  _clock = node->get_clock();
  _costmap = costmap_ros->getCostmap();
  _costmap_ros = costmap_ros;
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();
  _raw_plan_publisher = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);

  RCLCPP_INFO(_logger, "Configuring %s of type SmacPlannerLattice", name.c_str());

  // General planner params
  double analytic_expansion_max_length_m;
  bool smooth_path;

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".tolerance", rclcpp::ParameterValue(0.25));
  _tolerance = static_cast<float>(node->get_parameter(name + ".tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", _allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_iterations", rclcpp::ParameterValue(1000000));
  node->get_parameter(name + ".max_iterations", _max_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_on_approach_iterations", rclcpp::ParameterValue(1000));
  node->get_parameter(name + ".max_on_approach_iterations", _max_on_approach_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_path", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".smooth_path", smooth_path);

  // Default to a well rounded model: 16 bin, 0.4m turning radius, ackermann model
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lattice_filepath", rclcpp::ParameterValue(
      ament_index_cpp::get_package_share_directory("nav2_smac_planner") +
      "/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann/output.json"));
  node->get_parameter(name + ".lattice_filepath", _search_info.lattice_filepath);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cache_obstacle_heuristic", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".cache_obstacle_heuristic", _search_info.cache_obstacle_heuristic);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".reverse_penalty", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".reverse_penalty", _search_info.reverse_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".change_penalty", rclcpp::ParameterValue(0.05));
  node->get_parameter(name + ".change_penalty", _search_info.change_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".non_straight_penalty", rclcpp::ParameterValue(1.05));
  node->get_parameter(name + ".non_straight_penalty", _search_info.non_straight_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cost_penalty", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".cost_penalty", _search_info.cost_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".retrospective_penalty", rclcpp::ParameterValue(0.015));
  node->get_parameter(name + ".retrospective_penalty", _search_info.retrospective_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".rotation_penalty", rclcpp::ParameterValue(5.0));
  node->get_parameter(name + ".rotation_penalty", _search_info.rotation_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_ratio", rclcpp::ParameterValue(3.5));
  node->get_parameter(name + ".analytic_expansion_ratio", _search_info.analytic_expansion_ratio);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_max_length", rclcpp::ParameterValue(3.0));
  node->get_parameter(name + ".analytic_expansion_max_length", analytic_expansion_max_length_m);
  _search_info.analytic_expansion_max_length =
    analytic_expansion_max_length_m / _costmap->getResolution();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_planning_time", rclcpp::ParameterValue(5.0));
  node->get_parameter(name + ".max_planning_time", _max_planning_time);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lookup_table_size", rclcpp::ParameterValue(20.0));
  node->get_parameter(name + ".lookup_table_size", _lookup_table_size);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_reverse_expansion", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".allow_reverse_expansion", _search_info.allow_reverse_expansion);

  _metadata = LatticeMotionTable::getLatticeMetadata(_search_info.lattice_filepath);
  _search_info.minimum_turning_radius =
    _metadata.min_turning_radius / (_costmap->getResolution());
  _motion_model = MotionModel::STATE_LATTICE;

  if (_max_on_approach_iterations <= 0) {
    RCLCPP_INFO(
      _logger, "On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");
    _max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (_max_iterations <= 0) {
    RCLCPP_INFO(
      _logger, "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    _max_iterations = std::numeric_limits<int>::max();
  }

  float lookup_table_dim =
    static_cast<float>(_lookup_table_size) /
    static_cast<float>(_costmap->getResolution());

  // Make sure its a whole number
  lookup_table_dim = static_cast<float>(static_cast<int>(lookup_table_dim));

  // Make sure its an odd number
  if (static_cast<int>(lookup_table_dim) % 2 == 0) {
    RCLCPP_INFO(
      _logger,
      "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
      lookup_table_dim);
    lookup_table_dim += 1.0;
  }

  // Initialize collision checker using 72 evenly sized bins instead of the lattice
  // heading angles. This is done so that we have precomputed angles every 5 degrees.
  // If we used the sparse lattice headings (usually 16), then when we attempt to collision
  // check for intermediary points of the primitives, we're forced to round to one of the 16
  // increments causing "wobbly" checks that could cause larger robots to virtually show collisions
  // in valid configurations. This approximation helps to bound orientation error for all checks
  // in exchange for slight inaccuracies in the collision headings in terminal search states.
  _collision_checker = GridCollisionChecker(_costmap, 72u, node);
  _collision_checker.setFootprint(
    costmap_ros->getRobotFootprint(),
    costmap_ros->getUseRadius(),
    findCircumscribedCost(costmap_ros));

  // Initialize A* template
  _a_star = std::make_unique<AStarAlgorithm<NodeLattice>>(_motion_model, _search_info);
  _a_star->initialize(
    _allow_unknown,
    _max_iterations,
    _max_on_approach_iterations,
    _max_planning_time,
    lookup_table_dim,
    _metadata.number_of_headings);

  // Initialize path smoother
  if (smooth_path) {
    SmootherParams params;
    params.get(node, name);
    _smoother = std::make_unique<Smoother>(params);
    _smoother->initialize(_metadata.min_turning_radius);
  }

  RCLCPP_INFO(
    _logger, "Configured plugin %s of type SmacPlannerLattice with "
    "maximum iterations %i, max on approach iterations %i, "
    "and %s. Tolerance %.2f. Using motion model: %s. State lattice file: %s.",
    _name.c_str(), _max_iterations, _max_on_approach_iterations,
    _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    _tolerance, toString(_motion_model).c_str(), _search_info.lattice_filepath.c_str());
}

void SmacPlannerLattice::activate()
{
  RCLCPP_INFO(
    _logger, "Activating plugin %s of type SmacPlannerLattice",
    _name.c_str());
  _raw_plan_publisher->on_activate();
  auto node = _node.lock();
  // Add callback for dynamic parameters
  _dyn_params_handler = node->add_on_set_parameters_callback(
    std::bind(&SmacPlannerLattice::dynamicParametersCallback, this, std::placeholders::_1));
}

void SmacPlannerLattice::deactivate()
{
  RCLCPP_INFO(
    _logger, "Deactivating plugin %s of type SmacPlannerLattice",
    _name.c_str());
  _raw_plan_publisher->on_deactivate();
  _dyn_params_handler.reset();
}

void SmacPlannerLattice::cleanup()
{
  RCLCPP_INFO(
    _logger, "Cleaning up plugin %s of type SmacPlannerLattice",
    _name.c_str());
  _a_star.reset();
  _smoother.reset();
  _raw_plan_publisher.reset();
}

// added by hz
// TODO:需要修改判断不可达的条件
// bool SmacPlannerLattice::isAreaFree(nav2_costmap_2d::Costmap2D* costmap, int mx, int my) {
//     for (int dx = -5; dx <= 5; ++dx) {
//         for (int dy = -5; dy <= 5; ++dy) {
//             int check_mx = mx + dx;
//             int check_my = my + dy;

//             if (check_mx >= 0 && check_mx < static_cast<int>(costmap->getSizeInCellsX()) &&
//                 check_my >= 0 && check_my < static_cast<int>(costmap->getSizeInCellsY())) {
//                 if (costmap->getCost(check_mx, check_my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
//                     return false;
//                 }
//             } else {
//                 return false;
//             }
//         }
//     }
//     return true;
// }

// version 1
// bool SmacPlannerLattice::isAreaFree(nav2_costmap_2d::Costmap2D* costmap, const geometry_msgs::msg::PoseStamped& goal) {
//     // 车的四个顶点相对于车中心的坐标
//     std::vector<std::pair<double, double>> vertices = {
//         {0.7, 0.35}, {0.7, -0.35}, {-0.55, -0.35}, {-0.55, 0.35}
//     };

//     // 提取目标点的位置信息
//     double goal_x = goal.pose.position.x;
//     double goal_y = goal.pose.position.y;

//     // 提取目标点的四元数并转换为欧拉角
//     tf2::Quaternion q(
//         goal.pose.orientation.x,
//         goal.pose.orientation.y,
//         goal.pose.orientation.z,
//         goal.pose.orientation.w);
//     tf2::Matrix3x3 m(q);
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);

//     // 遍历长方形区域内的所有点
//     for (double x = -0.55; x <= 0.7; x += costmap->getResolution()) {
//         for (double y = -0.35; y <= 0.35; y += costmap->getResolution()) {
//             // 旋转点坐标
//             double rotated_x = x * cos(yaw) - y * sin(yaw);
//             double rotated_y = x * sin(yaw) + y * cos(yaw);

//             // 将旋转后的点坐标转换到世界坐标系
//             double world_x = goal_x + rotated_x;
//             double world_y = goal_y + rotated_y;

//             // 将世界坐标转换到栅格地图坐标
//             unsigned int map_x, map_y;
//             if (!costmap->worldToMap(world_x, world_y, map_x, map_y)) {
//                 return false; // 如果点在地图外，认为有障碍物
//             }

//             // 检查点的代价值
//             if (costmap->getCost(map_x, map_y) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
//                 return false; // 如果点的代价值大于等于 LETHAL_OBSTACLE，认为有障碍物
//             }
//         }
//     }
//     return true; // 如果所有点都没有障碍物，认为区域无障碍物
//  }

// version 2
// TODO:这个方式会靠障碍物很远
// TODO:如果local costmap有障碍物，而global costmap没有显示障碍物，该功能无法生效
bool SmacPlannerLattice::isAreaFree(nav2_costmap_2d::Costmap2D* costmap, const geometry_msgs::msg::PoseStamped& goal) {
    // 车的四个顶点相对于车中心的坐标
    std::vector<std::pair<double, double>> vertices = {
        {0.7, 0.35}, {0.7, -0.35}, {-0.55, -0.35}, {-0.55, 0.35}
    };

    // 提取目标点的位置信息
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;

    // 提取目标点的四元数并转换为欧拉角
    tf2::Quaternion q(
        goal.pose.orientation.x,
        goal.pose.orientation.y,
        goal.pose.orientation.z,
        goal.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // 遍历长方形区域内的所有点
    for (double x = -0.45; x <= 0.6; x += costmap->getResolution()) {
        for (double y = -0.25; y <= 0.25; y += costmap->getResolution()) {
            // 旋转点坐标
            double rotated_x = x * cos(yaw) - y * sin(yaw);
            double rotated_y = x * sin(yaw) + y * cos(yaw);

            // 将旋转后的点坐标转换到世界坐标系
            double world_x = goal_x + rotated_x;
            double world_y = goal_y + rotated_y;

            // 将世界坐标转换到栅格地图坐标
            unsigned int map_x, map_y;
            if (!costmap->worldToMap(world_x, world_y, map_x, map_y)) {
                return false; // 如果点在地图外，认为有障碍物
            }

            // 检查点的代价值
            if (costmap->getCost(map_x, map_y) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) { // >= nav2_costmap_2d::LETHAL_OBSTACLE
                return false; // 如果点的代价值大于等于 LETHAL_OBSTACLE，认为有障碍物
            }
        }
    }
    return true; // 如果所有点都没有障碍物，认为区域无障碍物
}
// end
nav_msgs::msg::Path SmacPlannerLattice::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::lock_guard<std::mutex> lock_reinit(_mutex);
  steady_clock::time_point a = steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Set collision checker and costmap information
  _collision_checker.setFootprint(
    _costmap_ros->getRobotFootprint(),
    _costmap_ros->getUseRadius(),
    findCircumscribedCost(_costmap_ros));
  _a_star->setCollisionChecker(&_collision_checker);

  // Set starting point, in A* bin search coordinates
  unsigned int mx, my;
  _costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  _a_star->setStart(
    mx, my,
    NodeLattice::motion_table.getClosestAngularBin(tf2::getYaw(start.pose.orientation)));

  // Set goal point, in A* bin search coordinates
  _costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);

  // added by hz
  // version 1
  // goal checker
  // 定义方向数组，用于遍历八领域的相邻网格
  // const int directions[8][2] = {
  //   {1, 0}, {-1, 0}, {0, 1}, {0, -1}, 
  //   {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  // };

  // if (!isAreaFree(_costmap, goal)) {
  //   RCLCPP_WARN(
  //     _logger,
  //     "Goal is in collision, searching for nearest free space.");

  //   // 使用队列进行广度优先搜索
  //   std::queue<std::tuple<unsigned int, unsigned int>> q;
  //   q.push(std::make_tuple(mx, my));

  //   // 使用 unordered_set 标记已访问的节点
  //   std::unordered_set<unsigned int> visited;
  //   visited.insert(mx * _costmap->getSizeInCellsY() + my);

  //   bool found = false;
  //   unsigned int new_mx = mx, new_my = my;

  //   while (!q.empty() && !found) {
  //       auto [current_mx, current_my] = q.front();
  //       q.pop();

  //       // 遍历相邻的网格
  //       for (const auto& direction : directions) {
  //           int next_mx = static_cast<int>(current_mx) + direction[0];
  //           int next_my = static_cast<int>(current_my) + direction[1];

  //           // 检查边界条件
  //           if (next_mx >= 0 && next_mx < static_cast<int>(_costmap->getSizeInCellsX()) &&
  //               next_my >= 0 && next_my < static_cast<int>(_costmap->getSizeInCellsY())) {
  //               unsigned int next_index = next_mx * _costmap->getSizeInCellsY() + next_my;
  //               if (visited.find(next_index) == visited.end()) {
  //                   visited.insert(next_index);
  //                   auto next_value = _costmap->getCost(next_mx, next_my);

  //                   // 如果找到自由空间
  //                   if (next_value < nav2_costmap_2d::MAX_NON_OBSTACLE) {
  //                       bool area_free = true;

  //                       // 检查以找到的点为中心的 10*10 区域
  //                       for (int dx = -5; dx <= 5 && area_free; ++dx) {
  //                           for (int dy = -5; dy <= 5; ++dy) {
  //                               int check_mx = next_mx + dx;
  //                               int check_my = next_my + dy;

  //                               if (check_mx >= 0 && check_mx < static_cast<int>(_costmap->getSizeInCellsX()) &&
  //                                   check_my >= 0 && check_my < static_cast<int>(_costmap->getSizeInCellsY())) {
  //                                   if (_costmap->getCost(check_mx, check_my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
  //                                       area_free = false;
  //                                       break;
  //                                   }
  //                               } else {
  //                                   area_free = false;
  //                                   break;
  //                               }
  //                           }
  //                       }

  //                       if (area_free) {
  //                           new_mx = next_mx;
  //                           new_my = next_my;
  //                           found = true;
  //                           break;
  //                       }
  //                   }

  //                   // 将相邻的网格加入队列
  //                   q.push(std::make_tuple(next_mx, next_my));
  //               }
  //           }
  //       }
  //   }
  //   if (found) {
  //     RCLCPP_INFO(
  //       _logger,
  //       "Found nearest free space at (%u, %u).", new_mx, new_my);
  //     mx = new_mx;
  //     my = new_my;
  //   } else {
  //     RCLCPP_ERROR(
  //       _logger,
  //       "No free space found near the goal.");
  //   }
  // }
  // end

  // added by hz
  // version 2
  // TODO:会出现一个问题，如果定位出现问题，它依然会寻找一个可达点，可能会乱跑
  // goal checker
  // 定义方向数组，用于遍历八领域的相邻网格
  geometry_msgs::msg::PoseStamped goal_new = goal;

  const int directions[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, 
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };

  if (!isAreaFree(_costmap, goal)) {
    RCLCPP_WARN(
      _logger,
      "Goal is in collision, searching for nearest free space.");

    // 使用队列进行广度优先搜索
    std::queue<std::tuple<unsigned int, unsigned int>> q;
    q.push(std::make_tuple(mx, my));

    // 使用 unordered_set 标记已访问的节点
    std::unordered_set<unsigned int> visited;
    visited.insert(mx * _costmap->getSizeInCellsY() + my);

    bool found = false;
    unsigned int new_mx = mx, new_my = my;

    while (!q.empty() && !found) {
        auto [current_mx, current_my] = q.front();
        q.pop();

        // 遍历相邻的网格
        for (const auto& direction : directions) {
            int next_mx = static_cast<int>(current_mx) + direction[0];
            int next_my = static_cast<int>(current_my) + direction[1];

            // 检查边界条件
            if (next_mx >= 0 && next_mx < static_cast<int>(_costmap->getSizeInCellsX()) &&
                next_my >= 0 && next_my < static_cast<int>(_costmap->getSizeInCellsY())) {
                unsigned int next_index = next_mx * _costmap->getSizeInCellsY() + next_my;
                if (visited.find(next_index) == visited.end()) {
                    visited.insert(next_index);
                    auto next_value = _costmap->getCost(next_mx, next_my);

                    // 如果找到自由空间
                    if (next_value <= nav2_costmap_2d::MAX_NON_OBSTACLE) {
                        bool area_free = true;
                        
                        _costmap->mapToWorld(next_mx, next_my, goal_new.pose.position.x, goal_new.pose.position.y);
                        area_free = isAreaFree(_costmap, goal_new);

                        if (area_free) {
                            new_mx = next_mx;
                            new_my = next_my;
                            found = true;
                            break;
                        }
                    }

                    // 将相邻的网格加入队列
                    q.push(std::make_tuple(next_mx, next_my));
                }
            }
        }
    }
    if (found) {
      RCLCPP_INFO(
        _logger,
        "Found nearest free space at (%u, %u).", new_mx, new_my);
      mx = new_mx;
      my = new_my;
    } else {
      RCLCPP_ERROR(
        _logger,
        "No free space found near the goal.");
    }
  }
  // end

  _a_star->setGoal(
    mx, my,
    NodeLattice::motion_table.getClosestAngularBin(tf2::getYaw(goal.pose.orientation)));

  // Setup message
  nav_msgs::msg::Path plan;
  plan.header.stamp = _clock->now();
  plan.header.frame_id = _global_frame;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // Compute plan
  NodeLattice::CoordinateVector path;
  int num_iterations = 0;
  std::string error;

  // added by hz
  auto start_time = std::chrono::high_resolution_clock::now();
  // end
  try {
    if (!_a_star->createPath(
        path, num_iterations, _tolerance / static_cast<float>(_costmap->getResolution())))
    {
      if (num_iterations < _a_star->getMaxIterations()) {
        error = std::string("no valid path found");
      } else {
        error = std::string("exceeded maximum iterations");
      }
    }
  } catch (const std::runtime_error & e) {
    error = "invalid use: ";
    error += e.what();
  }

  // added by hz
  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_time = end_time - start_time;

  RCLCPP_INFO(
    _logger,
    "Path creation took %.6f seconds.",
    elapsed_time.count());
  // end

  if (!error.empty()) {
    RCLCPP_WARN(
      _logger,
      "%s: failed to create plan, %s.",
      _name.c_str(), error.c_str());
    return plan;
  }

  // Convert to world coordinates
  plan.poses.reserve(path.size());
  geometry_msgs::msg::PoseStamped last_pose = pose;
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = getWorldCoords(path[i].x, path[i].y, _costmap);
    pose.pose.orientation = getWorldOrientation(path[i].theta);
    if (fabs(pose.pose.position.x - last_pose.pose.position.x) < 1e-4 &&
      fabs(pose.pose.position.y - last_pose.pose.position.y) < 1e-4 &&
      fabs(tf2::getYaw(pose.pose.orientation) - tf2::getYaw(last_pose.pose.orientation)) < 1e-4)
    {
      RCLCPP_DEBUG(
        _logger,
        "Removed a path from the path due to replication. "
        "Make sure your minimum control set does not contain duplicate values!");
      continue;
    }
    last_pose = pose;
    plan.poses.push_back(pose);
  }

  // Publish raw path for debug
  if (_raw_plan_publisher->get_subscription_count() > 0) {
    _raw_plan_publisher->publish(plan);
  }

  // Find how much time we have left to do smoothing
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  double time_remaining = _max_planning_time - static_cast<double>(time_span.count());

#ifdef BENCHMARK_TESTING
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif

  // Smooth plan
  if (_smoother && num_iterations > 1) {
    _smoother->smooth(plan, _costmap, time_remaining);
  }

#ifdef BENCHMARK_TESTING
  steady_clock::time_point c = steady_clock::now();
  duration<double> time_span2 = duration_cast<duration<double>>(c - b);
  std::cout << "It took " << time_span2.count() * 1000 <<
    " milliseconds to smooth path." << std::endl;
#endif

  return plan;
}

rcl_interfaces::msg::SetParametersResult
SmacPlannerLattice::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(_mutex);

  bool reinit_a_star = false;
  bool reinit_smoother = false;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == _name + ".max_planning_time") {
        reinit_a_star = true;
        _max_planning_time = parameter.as_double();
      } else if (name == _name + ".tolerance") {
        _tolerance = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".lookup_table_size") {
        reinit_a_star = true;
        _lookup_table_size = parameter.as_double();
      } else if (name == _name + ".reverse_penalty") {
        reinit_a_star = true;
        _search_info.reverse_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".change_penalty") {
        reinit_a_star = true;
        _search_info.change_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".non_straight_penalty") {
        reinit_a_star = true;
        _search_info.non_straight_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".cost_penalty") {
        reinit_a_star = true;
        _search_info.cost_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".rotation_penalty") {
        reinit_a_star = true;
        _search_info.rotation_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".analytic_expansion_ratio") {
        reinit_a_star = true;
        _search_info.analytic_expansion_ratio = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".analytic_expansion_max_length") {
        reinit_a_star = true;
        _search_info.analytic_expansion_max_length =
          static_cast<float>(parameter.as_double()) / _costmap->getResolution();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == _name + ".allow_unknown") {
        reinit_a_star = true;
        _allow_unknown = parameter.as_bool();
      } else if (name == _name + ".cache_obstacle_heuristic") {
        reinit_a_star = true;
        _search_info.cache_obstacle_heuristic = parameter.as_bool();
      } else if (name == _name + ".allow_reverse_expansion") {
        reinit_a_star = true;
        _search_info.allow_reverse_expansion = parameter.as_bool();
      } else if (name == _name + ".smooth_path") {
        if (parameter.as_bool()) {
          reinit_smoother = true;
        } else {
          _smoother.reset();
        }
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == _name + ".max_iterations") {
        reinit_a_star = true;
        _max_iterations = parameter.as_int();
        if (_max_iterations <= 0) {
          RCLCPP_INFO(
            _logger, "maximum iteration selected as <= 0, "
            "disabling maximum iterations.");
          _max_iterations = std::numeric_limits<int>::max();
        }
      }
    } else if (name == _name + ".max_on_approach_iterations") {
      reinit_a_star = true;
      _max_on_approach_iterations = parameter.as_int();
      if (_max_on_approach_iterations <= 0) {
        RCLCPP_INFO(
          _logger, "On approach iteration selected as <= 0, "
          "disabling tolerance and on approach iterations.");
        _max_on_approach_iterations = std::numeric_limits<int>::max();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == _name + ".lattice_filepath") {
        reinit_a_star = true;
        if (_smoother) {
          reinit_smoother = true;
        }
        _search_info.lattice_filepath = parameter.as_string();
        _metadata = LatticeMotionTable::getLatticeMetadata(_search_info.lattice_filepath);
        _search_info.minimum_turning_radius =
          _metadata.min_turning_radius / (_costmap->getResolution());
      }
    }
  }

  // Re-init if needed with mutex lock (to avoid re-init while creating a plan)
  if (reinit_a_star || reinit_smoother) {
    // convert to grid coordinates
    _search_info.minimum_turning_radius =
      _metadata.min_turning_radius / (_costmap->getResolution());
    float lookup_table_dim =
      static_cast<float>(_lookup_table_size) /
      static_cast<float>(_costmap->getResolution());

    // Make sure its a whole number
    lookup_table_dim = static_cast<float>(static_cast<int>(lookup_table_dim));

    // Make sure its an odd number
    if (static_cast<int>(lookup_table_dim) % 2 == 0) {
      RCLCPP_INFO(
        _logger,
        "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
        lookup_table_dim);
      lookup_table_dim += 1.0;
    }

    // Re-Initialize smoother
    if (reinit_smoother) {
      auto node = _node.lock();
      SmootherParams params;
      params.get(node, _name);
      _smoother = std::make_unique<Smoother>(params);
      _smoother->initialize(_metadata.min_turning_radius);
    }

    // Re-Initialize A* template
    if (reinit_a_star) {
      _a_star = std::make_unique<AStarAlgorithm<NodeLattice>>(_motion_model, _search_info);
      _a_star->initialize(
        _allow_unknown,
        _max_iterations,
        _max_on_approach_iterations,
        _max_planning_time,
        lookup_table_dim,
        _metadata.number_of_headings);
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_smac_planner::SmacPlannerLattice, nav2_core::GlobalPlanner)
