#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <numeric>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "rclcpp/parameter_map.hpp"

#include "utils/factory.hpp"

namespace
{

struct RosContext
{
  RosContext() {rclcpp::init(0, nullptr);}
  ~RosContext() {rclcpp::shutdown();}
} g_ros_context;

struct ScenarioResult
{
  struct CostmapSnapshot
  {
    unsigned int size_x{0};
    unsigned int size_y{0};
    double resolution{0.0};
    double origin_x{0.0};
    double origin_y{0.0};
    double footprint_half_x{0.0};
    double footprint_half_y{0.0};
    double inflation_radius{0.0};
    double cost_scaling_factor{0.0};
    std::vector<unsigned char> costs;
  } costmap;
  nav_msgs::msg::Path path;
  mppi::models::Trajectories trajectories;
  mppi::models::Vector weights;
  mppi::models::Matrix optimized_trajectory;
};

struct WeightStats
{
  double top1{0.0};
  double top5{0.0};
  double top10{0.0};
  double top20{0.0};
  double normalized_entropy{0.0};
  double effective_sample_size{0.0};
  size_t above_one_ppm{0};
};

double optimizedPathRmse(const ScenarioResult & result)
{
  if (result.optimized_trajectory.rows() == 0 || result.path.poses.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double squared_error = 0.0;
  for (Eigen::Index i = 0; i < result.optimized_trajectory.rows(); ++i) {
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto & pose : result.path.poses) {
      const double dx = result.optimized_trajectory(i, 0) - pose.pose.position.x;
      const double dy = result.optimized_trajectory(i, 1) - pose.pose.position.y;
      nearest = std::min(nearest, dx * dx + dy * dy);
    }
    squared_error += nearest;
  }
  return std::sqrt(squared_error / static_cast<double>(result.optimized_trajectory.rows()));
}

WeightStats analyzeAndWriteWeights(
  const ScenarioResult & result, const std::string & csv_filename)
{
  std::vector<float> sorted(
    result.weights.data(), result.weights.data() + result.weights.size());
  std::sort(sorted.begin(), sorted.end(), std::greater<float>());
  WeightStats stats;
  double cumulative = 0.0;
  double entropy = 0.0;
  double squared_sum = 0.0;
  std::ofstream csv(csv_filename);
  csv << "rank,weight,cumulative,relative_to_top\n";
  for (size_t rank = 0; rank < sorted.size(); ++rank) {
    const double weight = sorted[rank];
    cumulative += weight;
    squared_sum += weight * weight;
    if (weight > 0.0) {
      entropy -= weight * std::log(weight);
    }
    if (weight >= 1e-6) {
      ++stats.above_one_ppm;
    }
    csv << rank + 1 << ',' << std::scientific << std::setprecision(9) << weight << ','
        << cumulative << ',' << weight / std::max<double>(sorted.front(), 1e-30) << '\n';
    if (rank == 0) {stats.top1 = cumulative;}
    if (rank == 4) {stats.top5 = cumulative;}
    if (rank == 9) {stats.top10 = cumulative;}
    if (rank == 19) {stats.top20 = cumulative;}
  }
  stats.normalized_entropy = entropy / std::log(static_cast<double>(sorted.size()));
  stats.effective_sample_size = 1.0 / squared_sum;
  return stats;
}

template<typename ScaleX, typename ScaleY>
void writeCostmap(
  std::ofstream & out, const ScenarioResult::CostmapSnapshot & costmap,
  ScaleX scale_x, ScaleY scale_y, double pixels_per_metre)
{
  const double cell_pixels = costmap.resolution * pixels_per_metre + 0.15;
  for (unsigned int my = 0; my < costmap.size_y; ++my) {
    for (unsigned int mx = 0; mx < costmap.size_x; ++mx) {
      const auto cost = costmap.costs[static_cast<size_t>(my) * costmap.size_x + mx];
      // Keep the report focused on trajectory selection: show lethal geometry only,
      // while the full inflated field remains active in critic scoring.
      if (cost < nav2_costmap_2d::LETHAL_OBSTACLE) {
        continue;
      }
      const double world_x = costmap.origin_x + mx * costmap.resolution;
      const double world_y = costmap.origin_y + my * costmap.resolution;
      out << "<rect x=\"" << scale_x(world_x) << "\" y=\""
          << scale_y(world_y + costmap.resolution) << "\" width=\"" << cell_pixels
          << "\" height=\"" << cell_pixels << "\" fill=\"#303030\"/>";
    }
  }
}

template<typename ScaleX, typename ScaleY>
void writeAxes(
  std::ofstream & out, ScaleX scale_x, ScaleY scale_y,
  double x_min, double x_max, double x_step,
  double y_min, double y_max, double y_step, bool compact = false)
{
  const double tick = compact ? 4.0 : 6.0;
  const int font_size = compact ? 9 : 11;
  out << "<g stroke=\"#777\" stroke-width=\"1\" fill=\"#555\" font-size=\""
      << font_size << "\">";
  for (double x = x_min; x <= x_max + 1e-6; x += x_step) {
    out << "<line x1=\"" << scale_x(x) << "\" y1=\"" << scale_y(y_min)
        << "\" x2=\"" << scale_x(x) << "\" y2=\"" << scale_y(y_min) + tick << "\"/>"
        << "<text stroke=\"none\" text-anchor=\"middle\" x=\"" << scale_x(x)
        << "\" y=\"" << scale_y(y_min) + tick + font_size << "\">"
        << std::fixed << std::setprecision(compact ? 0 : 1) << x << "</text>";
  }
  for (double y = y_min; y <= y_max + 1e-6; y += y_step) {
    out << "<line x1=\"" << scale_x(x_min) - tick << "\" y1=\"" << scale_y(y)
        << "\" x2=\"" << scale_x(x_min) << "\" y2=\"" << scale_y(y) << "\"/>"
        << "<text stroke=\"none\" text-anchor=\"end\" dominant-baseline=\"middle\" x=\""
        << scale_x(x_min) - tick - 2.0 << "\" y=\"" << scale_y(y) << "\">"
        << std::fixed << std::setprecision(1) << y << "</text>";
  }
  out << "<text stroke=\"none\" x=\"" << scale_x(x_max) + 8.0 << "\" y=\""
      << scale_y(y_min) + font_size << "\">x/m</text>"
      << "<text stroke=\"none\" x=\"" << scale_x(x_min) - 22.0 << "\" y=\""
      << scale_y(y_max) - 7.0 << "\">y/m</text></g>";
}

enum class Scene
{
  Curve,
  Obstacle,
  ObstacleAtOneMetre,
  LeftNarrowRightWide,
  NearGoalHalfMetre,
  ObstacleAtThreePointFiveMetres
};

struct CostmapConfig
{
  double width{0.0};
  double height{0.0};
  double resolution{0.0};
  double inflation_radius{0.0};
  double cost_scaling_factor{0.0};
  double footprint_half_x{0.0};
  double footprint_half_y{0.0};
};

ScenarioResult::CostmapSnapshot snapshotCostmap(
  const nav2_costmap_2d::Costmap2D & costmap, const CostmapConfig & config)
{
  ScenarioResult::CostmapSnapshot snapshot;
  snapshot.size_x = costmap.getSizeInCellsX();
  snapshot.size_y = costmap.getSizeInCellsY();
  snapshot.resolution = costmap.getResolution();
  snapshot.origin_x = costmap.getOriginX();
  snapshot.origin_y = costmap.getOriginY();
  snapshot.footprint_half_x = config.footprint_half_x;
  snapshot.footprint_half_y = config.footprint_half_y;
  snapshot.inflation_radius = config.inflation_radius;
  snapshot.cost_scaling_factor = config.cost_scaling_factor;
  const size_t cells = static_cast<size_t>(snapshot.size_x) * snapshot.size_y;
  snapshot.costs.assign(costmap.getCharMap(), costmap.getCharMap() + cells);
  return snapshot;
}

const rclcpp::Parameter & requireParameter(
  const rclcpp::ParameterMap & parameter_map, const std::string & node,
  const std::string & name)
{
  const auto node_it = parameter_map.find(node);
  if (node_it == parameter_map.end()) {
    throw std::runtime_error("Node '" + node + "' missing in W1 parameter file");
  }
  const auto parameter_it = std::find_if(
    node_it->second.begin(), node_it->second.end(),
    [&name](const auto & parameter) {return parameter.get_name() == name;});
  if (parameter_it == node_it->second.end()) {
    throw std::runtime_error("Parameter '" + node + "." + name + "' missing in W1 parameter file");
  }
  return *parameter_it;
}

double numericParameter(const rclcpp::Parameter & parameter)
{
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(parameter.as_int());
  }
  return parameter.as_double();
}

void setParameterOverride(
  std::vector<rclcpp::Parameter> & parameters, const std::string & name,
  const rclcpp::ParameterValue & value)
{
  parameters.erase(
    std::remove_if(
      parameters.begin(), parameters.end(),
      [&name](const auto & parameter) {return parameter.get_name() == name;}),
    parameters.end());
  parameters.emplace_back(name, value);
}

void loadW1TrackMppiParameters(
  const rclcpp::ParameterMap & parameter_map, const std::string & node_name,
  std::vector<rclcpp::Parameter> & parameters)
{
  constexpr const char * controller_node = "/controller_server";
  constexpr const char * mppi_prefix = "MPPIController.";
  const auto controller_it = parameter_map.find(controller_node);
  if (controller_it == parameter_map.end()) {
    throw std::runtime_error("Controller server parameters missing in W1 parameter file");
  }

  // Copy the complete MPPI configuration first. Optimizer ignores controller-only profile
  // selectors, while all sampling, motion, CUDA and critic subparameters stay identical to W1.
  for (const auto & parameter : controller_it->second) {
    const auto & source_name = parameter.get_name();
    if (source_name.rfind(mppi_prefix, 0) != 0) {
      continue;
    }
    const auto suffix = source_name.substr(std::char_traits<char>::length(mppi_prefix));
    setParameterOverride(
      parameters, node_name + "." + suffix, parameter.get_parameter_value());
  }

  setParameterOverride(
    parameters, "controller_frequency",
    requireParameter(parameter_map, controller_node, "controller_frequency").get_parameter_value());

  // The Behavior Tree selects MPPIController for FollowPath and ControllerServer invokes it in
  // TRACK. Apply the runtime stage_track velocity constraints that normally override MPPI's base
  // constraints inside MPPIController.
  for (const auto * field : {"vx_max", "vx_min", "vy_max", "wz_max"}) {
    setParameterOverride(
      parameters, node_name + "." + field,
      requireParameter(
        parameter_map, controller_node,
        std::string(mppi_prefix) + "stage_track." + field).get_parameter_value());
  }

  struct StageWeightBinding
  {
    const char * matrix_key;
    const char * optimizer_parameter;
  };
  static const StageWeightBinding bindings[] = {
    {"GoalCritic", "GoalCritic.cost_weight"},
    {"GoalLateralCritic", "GoalLateralCritic.cost_weight"},
    {"GoalLongitudinalCritic", "GoalLongitudinalCritic.cost_weight"},
    {"GoalAngleCritic", "GoalAngleCritic.cost_weight"},
    {"PathAngleCritic", "PathAngleCritic.cost_weight"},
    {"PathAlignCritic", "PathAlignCritic.cost_weight"},
    {"PathFollowCritic", "PathFollowCritic.cost_weight"},
    {"PreferForwardCritic", "PreferForwardCritic.cost_weight"},
    {"LinearVelocityCritic", "LinearVelocityCritic.cost_weight"},
    {"AngularVelocityCritic", "AngularVelocityCritic.cost_weight"},
    {"ObstaclesCritic", "ObstaclesCritic.repulsion_weight"},
    {"CostCritic", "CostCritic.cost_weight"},
  };
  for (const auto & binding : bindings) {
    const auto weights = requireParameter(
      parameter_map, controller_node,
      std::string(mppi_prefix) + "stage_weights." + binding.matrix_key).as_double_array();
    if (weights.size() != 3u) {
      throw std::runtime_error(
              std::string("stage_weights.") + binding.matrix_key +
              " must contain [align, track, approach]");
    }
    setParameterOverride(
      parameters, node_name + "." + binding.optimizer_parameter,
      rclcpp::ParameterValue(weights[1]));
  }

  // Deterministic test-only input; production regenerates stochastic noise normally.
  setParameterOverride(
    parameters, node_name + ".noise_seed", rclcpp::ParameterValue(int64_t{42}));
}

CostmapConfig loadW1CostmapConfig()
{
  const auto parameters = rclcpp::parameter_map_from_yaml_file(MPPI_TEST_W1_PARAMS_FILE);
  constexpr const char * node = "/local_costmap/local_costmap";
  CostmapConfig config;
  config.width = numericParameter(requireParameter(parameters, node, "width"));
  config.height = numericParameter(requireParameter(parameters, node, "height"));
  config.resolution = numericParameter(requireParameter(parameters, node, "resolution"));
  config.inflation_radius = numericParameter(
    requireParameter(parameters, node, "inflation_layer.inflation_radius"));
  config.cost_scaling_factor = numericParameter(
    requireParameter(parameters, node, "inflation_layer.cost_scaling_factor"));

  const auto footprint = requireParameter(parameters, node, "footprint").as_string();
  const std::regex number_pattern(R"([-+]?(?:\d*\.\d+|\d+\.?\d*))");
  for (auto it = std::sregex_iterator(footprint.begin(), footprint.end(), number_pattern);
    it != std::sregex_iterator(); ++it)
  {
    const double coordinate = std::abs(std::stod(it->str()));
    const auto index = static_cast<size_t>(std::distance(
      std::sregex_iterator(footprint.begin(), footprint.end(), number_pattern), it));
    if (index % 2 == 0) {
      config.footprint_half_x = std::max(config.footprint_half_x, coordinate);
    } else {
      config.footprint_half_y = std::max(config.footprint_half_y, coordinate);
    }
  }
  if (
    config.width <= 0.0 || config.height <= 0.0 || config.resolution <= 0.0 ||
    config.inflation_radius <= 0.0 || config.footprint_half_x <= 0.0 ||
    config.footprint_half_y <= 0.0)
  {
    throw std::runtime_error("Invalid local costmap values loaded from W1 parameter file");
  }
  return config;
}

std::shared_ptr<nav2_costmap_2d::Costmap2DROS> makeW1CostmapRos(
  const CostmapConfig & config, const std::string & footprint)
{
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("cost_map_node");
  auto results = costmap_ros->set_parameters({
    rclcpp::Parameter("width", static_cast<int>(std::round(config.width))),
    rclcpp::Parameter("height", static_cast<int>(std::round(config.height))),
    rclcpp::Parameter("resolution", config.resolution),
    rclcpp::Parameter("origin_x", -config.width / 2.0),
    rclcpp::Parameter("origin_y", -config.height / 2.0),
    rclcpp::Parameter("rolling_window", true),
    rclcpp::Parameter("footprint", footprint)});
  for (const auto & result : results) {
    if (!result.successful) {
      throw std::runtime_error("Failed to configure W1 costmap test fixture: " + result.reason);
    }
  }
  if (costmap_ros->on_configure(rclcpp_lifecycle::State{}) !=
    nav2_util::CallbackReturn::SUCCESS)
  {
    throw std::runtime_error("Failed to configure W1 Costmap2DROS test fixture");
  }

  // Inflation plugin parameters are declared during on_configure(). Setting them here invokes
  // the layer's dynamic parameter callback before critics inspect computeCost().
  results = costmap_ros->set_parameters({
    rclcpp::Parameter("inflation_layer.inflation_radius", config.inflation_radius),
    rclcpp::Parameter("inflation_layer.cost_scaling_factor", config.cost_scaling_factor)});
  for (const auto & result : results) {
    if (!result.successful) {
      throw std::runtime_error("Failed to configure W1 InflationLayer fixture: " + result.reason);
    }
  }
  return costmap_ros;
}

void paintInflatedRectangle(
  nav2_costmap_2d::Costmap2D * costmap, double min_x, double max_x,
  double min_y, double max_y, const CostmapConfig & config)
{
  for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); ++mx) {
    for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
      double x = 0.0;
      double y = 0.0;
      costmap->mapToWorld(mx, my, x, y);
      const double dx = std::max({min_x - x, 0.0, x - max_x});
      const double dy = std::max({min_y - y, 0.0, y - max_y});
      const double distance = std::hypot(dx, dy);
      if (distance <= 1e-6) {
        costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      } else if (distance < config.inflation_radius) {
        const double inscribed_radius = std::min(
          config.footprint_half_x, config.footprint_half_y);
        const auto inflated = static_cast<unsigned char>(
          distance <= inscribed_radius ? nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE :
          std::clamp(
            252.0 * std::exp(
              -config.cost_scaling_factor * (distance - inscribed_radius)),
            1.0, 252.0));
        costmap->setCost(mx, my, std::max(costmap->getCost(mx, my), inflated));
      }
    }
  }
}

nav_msgs::msg::Path makePath(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, Scene scene)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  path.header.stamp = node->now();
  const bool curved = scene == Scene::Curve;
  const bool one_metre_bypass = scene == Scene::ObstacleAtOneMetre;
  const int point_count = scene == Scene::NearGoalHalfMetre ? 10 : 80;
  for (int i = 0; i <= point_count; ++i) {
    const double x = 0.05 * i;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    double y = 0.0;
    double derivative = 0.0;
    if (curved) {
      y = 0.8 * (1.0 - std::cos(M_PI * x / 4.0));
      derivative = 0.2 * M_PI * std::sin(M_PI * x / 4.0);
    } else if (one_metre_bypass && x <= 2.0) {
      // This is the global path after obstacle-triggered replanning. It leaves the blocked
      // centre line immediately, clears the 1 m obstacle on the right, then merges back.
      y = -1.3 * std::sin(M_PI * x / 2.0);
      derivative = -1.3 * M_PI / 2.0 * std::cos(M_PI * x / 2.0);
    }
    pose.pose.position.y = y;
    const double yaw = std::atan(derivative);
    pose.pose.orientation.z = std::sin(0.5 * yaw);
    pose.pose.orientation.w = std::cos(0.5 * yaw);
    path.poses.push_back(pose);
  }
  return path;
}

ScenarioResult runScenario(const std::string & node_name, Scene scene)
{
  const auto yaml_parameters = rclcpp::parameter_map_from_yaml_file(MPPI_TEST_W1_PARAMS_FILE);
  constexpr const char * costmap_node = "/local_costmap/local_costmap";
  const auto costmap_config = loadW1CostmapConfig();
  const auto footprint = requireParameter(yaml_parameters, costmap_node, "footprint").as_string();
  std::vector<rclcpp::Parameter> params;
  loadW1TrackMppiParameters(yaml_parameters, node_name, params);
  const bool near_goal = scene == Scene::NearGoalHalfMetre;

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, options);
  auto costmap_ros = makeW1CostmapRos(costmap_config, footprint);

  if (scene == Scene::Obstacle || scene == Scene::LeftNarrowRightWide) {
    paintInflatedRectangle(
      costmap_ros->getCostmap(), 1.8, 2.2, -0.45, 0.45, costmap_config);
  }
  if (scene == Scene::ObstacleAtOneMetre) {
    // A 0.4 m deep obstacle centred exactly 1.0 m in front of the robot.
    paintInflatedRectangle(
      costmap_ros->getCostmap(), 0.8, 1.2, -0.45, 0.45, costmap_config);
  }
  if (scene == Scene::ObstacleAtThreePointFiveMetres) {
    // The obstacle centre is exactly 3.5 m in front of the initial robot pose.
    // Keep the same dimensions as scenario 2 so the plots isolate obstacle distance.
    paintInflatedRectangle(
      costmap_ros->getCostmap(), 3.3, 3.7, -0.45, 0.45, costmap_config);
  }
  if (scene == Scene::LeftNarrowRightWide) {
    // Robot-forward coordinates: +y is left. This wall leaves an unusably narrow left gap,
    // while the right side remains open down to the costmap boundary.
    paintInflatedRectangle(
      costmap_ros->getCostmap(), 1.25, 2.8, 0.78, 1.15, costmap_config);
  }

  auto parameter_handler = std::make_unique<mppi::ParametersHandler>(node);
  auto optimizer = getDummyOptimizer(node, costmap_ros, parameter_handler.get());
  auto path = makePath(node, scene);
  auto pose = getDummyPointStamped(node, TestPose{0.0, 0.0});
  pose.header.frame_id = "odom";
  if (scene == Scene::ObstacleAtOneMetre) {
    // State immediately after the staged controller's in-place alignment to the replanned path.
    pose.pose.orientation = path.poses.front().pose.orientation;
  }
  auto velocity = getDummyTwist();
  // The 1 m scene represents the instant immediately after the controller has completed its
  // in-place heading alignment. The robot is stationary at that hand-off; starting the optimizer
  // at 0.5 m/s instead makes emergency braking dominate and hides the intended bypass solution.
  velocity.linear.x = scene == Scene::ObstacleAtOneMetre ? 0.0 : (near_goal ? 0.2 : 0.5);
  // Production uses iteration_count=1 and preserves/shifts its nominal control sequence between
  // 10 Hz controller cycles. Warm the same Optimizer over several real single-iteration calls;
  // changing iteration_count to a test-only value would no longer represent the TRACK setup.
  constexpr size_t warmup_control_cycles = 4;
  for (size_t cycle = 0; cycle < warmup_control_cycles; ++cycle) {
    optimizer->evalControl(pose, velocity, path, nullptr);
  }
  return {
    snapshotCostmap(*costmap_ros->getCostmap(), costmap_config), path,
    optimizer->getGeneratedTrajectories(), optimizer->getTrajectoryWeights(),
    optimizer->getOptimizedTrajectory()};
}

void writeSvg(
  const ScenarioResult & result, const std::string & filename,
  const std::string & title, const WeightStats & stats)
{
  std::ofstream out(filename);
  const bool near_goal_view = !result.path.poses.empty() &&
    result.path.poses.back().pose.position.x <= 0.6;
  const double overview_scale = near_goal_view ? 900.0 : 135.0;
  auto sx = [overview_scale](double x) {return 65.0 + x * overview_scale;};
  auto sy = [overview_scale](double y) {return 420.0 - y * overview_scale;};
  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1420\" height=\"760\" "
         "viewBox=\"0 0 1420 760\"><rect width=\"100%\" height=\"100%\" fill=\"white\"/>";
  out << "<text x=\"25\" y=\"28\" font-size=\"20\">" << title << "</text>";
  const double path_rmse = optimizedPathRmse(result);
  out << "<text x=\"500\" y=\"28\" font-size=\"13\">Top1="
      << std::fixed << std::setprecision(2) << stats.top1 * 100.0 << "%  Top5="
      << stats.top5 * 100.0 << "%  Top20=" << stats.top20 * 100.0
      << "%  ESS=" << std::setprecision(1) << stats.effective_sample_size
      << "  entropy=" << std::setprecision(3) << stats.normalized_entropy
      << "  optimal-path RMSE=";
  if (std::isfinite(path_rmse)) {
    out << path_rmse << "m";
  } else {
    out << "N/A (near goal)";
  }
  out << "</text>";
  out << "<text x=\"25\" y=\"55\" font-size=\"16\" font-weight=\"bold\">Top-25 轨迹总览</text>";
  out << "<rect x=\"35\" y=\"70\" width=\"610\" height=\"610\" fill=\"#fafafa\" stroke=\"#bbb\"/>";
  writeCostmap(out, result.costmap, sx, sy, overview_scale);
  if (near_goal_view) {
    writeAxes(out, sx, sy, 0.0, 0.6, 0.1, -0.2, 0.3, 0.1);
  } else {
    writeAxes(out, sx, sy, 0.0, 4.0, 0.5, -1.5, 2.5, 0.5);
  }
  out << "<polyline fill=\"none\" stroke=\"black\" stroke-width=\"3\" points=\"";
  for (const auto & pose : result.path.poses) {
    out << sx(pose.pose.position.x) << ',' << sy(pose.pose.position.y) << ' ';
  }
  out << "\"/>";

  std::vector<Eigen::Index> order(static_cast<size_t>(result.weights.size()));
  std::iota(order.begin(), order.end(), Eigen::Index{0});
  const size_t count = std::min<size_t>(25, order.size());
  std::partial_sort(
    order.begin(), order.begin() + count, order.end(),
    [&](auto a, auto b) {return result.weights(a) > result.weights(b);});
  const float max_weight = std::max(result.weights(order.front()), 1e-12f);
  for (size_t rank = count; rank-- > 0;) {
    const auto row = order[rank];
    const float relative = result.weights(row) / max_weight;
    const int red = static_cast<int>(255.0f * relative);
    const int blue = static_cast<int>(255.0f * (1.0f - relative));
    out << "<polyline fill=\"none\" stroke=\"rgb(" << red << ",80," << blue
        << ")\" stroke-opacity=\"" << 0.25f + 0.75f * relative
        << "\" stroke-width=\"" << 0.8f + 3.2f * relative << "\" points=\"";
    for (Eigen::Index step = 0; step < result.trajectories.x.cols(); ++step) {
      out << sx(result.trajectories.x(row, step)) << ','
          << sy(result.trajectories.y(row, step)) << ' ';
    }
    out << "\"/>";
  }
  if (result.optimized_trajectory.rows() > 0) {
    out << "<polyline fill=\"none\" stroke=\"#ff8c00\" stroke-width=\"5\" points=\"";
    for (Eigen::Index i = 0; i < result.optimized_trajectory.rows(); ++i) {
      out << sx(result.optimized_trajectory(i, 0)) << ','
          << sy(result.optimized_trajectory(i, 1)) << ' ';
    }
    out << "\"/>";
  }

  // Split the six highest-weight trajectories into individual subplots.
  out << "<text x=\"690\" y=\"55\" font-size=\"16\" font-weight=\"bold\">高权重轨迹分解</text>";
  constexpr size_t subplot_count = 6;
  constexpr double panel_width = 330.0;
  constexpr double panel_height = 205.0;
  const double panel_scale_x = near_goal_view ? 400.0 : 67.0;
  const double panel_scale_y = near_goal_view ? 400.0 : 67.0;
  for (size_t rank = 0; rank < std::min(subplot_count, count); ++rank) {
    const auto row = order[rank];
    const size_t column = rank % 2;
    const size_t panel_row = rank / 2;
    const double panel_x = 690.0 + column * 355.0;
    const double panel_y = 70.0 + panel_row * 220.0;
    auto panel_sx = [panel_x, panel_scale_x](double x) {
        return panel_x + 35.0 + x * panel_scale_x;
      };
    auto panel_sy = [panel_y, panel_scale_y](double y) {
        return panel_y + 135.0 - y * panel_scale_y;
      };
    const float weight = result.weights(row);
    const float relative = weight / max_weight;

    out << "<rect x=\"" << panel_x << "\" y=\"" << panel_y << "\" width=\""
        << panel_width << "\" height=\"" << panel_height
        << "\" rx=\"6\" fill=\"#fafafa\" stroke=\"#aaa\"/>";
    out << "<text x=\"" << panel_x + 12.0 << "\" y=\"" << panel_y + 22.0
        << "\" font-size=\"14\" font-weight=\"bold\">#" << rank + 1
        << "  weight=" << std::scientific << std::setprecision(3) << weight
        << "  relative=" << std::fixed << std::setprecision(1) << relative * 100.0f
        << "%</text>";
    writeCostmap(out, result.costmap, panel_sx, panel_sy, panel_scale_x);
    if (near_goal_view) {
      writeAxes(out, panel_sx, panel_sy, 0.0, 0.6, 0.2, -0.2, 0.2, 0.1, true);
    } else {
      writeAxes(out, panel_sx, panel_sy, 0.0, 4.0, 1.0, -1.0, 1.5, 0.5, true);
    }
    out << "<polyline fill=\"none\" stroke=\"#999\" stroke-width=\"1.5\" points=\"";
    for (const auto & pose : result.path.poses) {
      out << panel_sx(pose.pose.position.x) << ',' << panel_sy(pose.pose.position.y) << ' ';
    }
    out << "\"/>";
    out << "<polyline fill=\"none\" stroke=\"#e31a1c\" stroke-width=\"3\" points=\"";
    for (Eigen::Index step = 0; step < result.trajectories.x.cols(); ++step) {
      out << panel_sx(result.trajectories.x(row, step)) << ','
          << panel_sy(result.trajectories.y(row, step)) << ' ';
    }
    out << "\"/>";
    if (result.optimized_trajectory.rows() > 0) {
      out << "<polyline fill=\"none\" stroke=\"#ff8c00\" stroke-width=\"3\" points=\"";
      for (Eigen::Index i = 0; i < result.optimized_trajectory.rows(); ++i) {
        out << panel_sx(result.optimized_trajectory(i, 0)) << ','
            << panel_sy(result.optimized_trajectory(i, 1)) << ' ';
      }
      out << "\"/>";
    }
  }
  out << "<text x=\"35\" y=\"716\" font-size=\"13\">深灰 = 致命障碍；黑线 = 参考路径；橙线 = 最终预测执行路径</text>";
  out << "<text x=\"690\" y=\"716\" font-size=\"13\">红线 = 候选轨迹；灰线 = 参考路径；橙线 = Optimal Trajectory</text>";
  out << "<text x=\"35\" y=\"742\" font-size=\"12\">W1 local costmap: "
      << std::fixed << std::setprecision(2)
      << result.costmap.size_x * result.costmap.resolution << "m × "
      << result.costmap.size_y * result.costmap.resolution << "m, resolution="
      << result.costmap.resolution << "m, footprint half-size="
      << result.costmap.footprint_half_x << " × " << result.costmap.footprint_half_y
      << "m, inflation=" << result.costmap.inflation_radius << "m, scale="
      << result.costmap.cost_scaling_factor << "</text>";
  out << "</svg>";
}

TEST(MppiTuningScenarios, SingleCurve)
{
  const auto result = runScenario("mppi_curve_tuning", Scene::Curve);
  const auto stats = analyzeAndWriteWeights(result, "/tmp/mppi_curve_weights.csv");
  writeSvg(result, "/tmp/mppi_curve_scenario.svg", "MPPI 场景一：单一曲线", stats);
}

TEST(MppiTuningScenarios, StraightPathObstacleAtTwoMeters)
{
  const auto result = runScenario("mppi_obstacle_tuning", Scene::Obstacle);
  const auto stats = analyzeAndWriteWeights(result, "/tmp/mppi_obstacle_2m_weights.csv");
  writeSvg(
    result, "/tmp/mppi_obstacle_2m_scenario.svg",
    "MPPI 场景二：直线路径，前方 2m 障碍物", stats);
}

TEST(MppiTuningScenarios, StraightPathObstacleAtOneMetre)
{
  const auto result = runScenario(
    "mppi_obstacle_1m_tuning", Scene::ObstacleAtOneMetre);
  const auto stats = analyzeAndWriteWeights(result, "/tmp/mppi_obstacle_1m_weights.csv");
  writeSvg(
    result, "/tmp/mppi_obstacle_1m_scenario.svg",
    "MPPI 新场景：原地对齐完成后，前方 1m 障碍物", stats);
}

TEST(MppiTuningScenarios, LeftNarrowRightWide)
{
  const auto result = runScenario("mppi_asymmetric_tuning", Scene::LeftNarrowRightWide);
  const auto stats = analyzeAndWriteWeights(
    result, "/tmp/mppi_left_narrow_right_wide_weights.csv");
  writeSvg(
    result, "/tmp/mppi_left_narrow_right_wide_scenario.svg",
    "MPPI 场景三：左窄右宽", stats);
}

TEST(MppiTuningScenarios, GoalAtHalfMetre)
{
  const auto result = runScenario("mppi_near_goal_tuning", Scene::NearGoalHalfMetre);
  const auto stats = analyzeAndWriteWeights(result, "/tmp/mppi_near_goal_05m_weights.csv");
  writeSvg(
    result, "/tmp/mppi_near_goal_05m_scenario.svg",
    "MPPI 场景四：目标点距离当前位置 0.5m", stats);
}

TEST(MppiTuningScenarios, StraightPathObstacleAtThreePointFiveMetres)
{
  const auto result = runScenario(
    "mppi_obstacle_35m_tuning", Scene::ObstacleAtThreePointFiveMetres);
  const auto stats = analyzeAndWriteWeights(result, "/tmp/mppi_obstacle_35m_weights.csv");
  writeSvg(
    result, "/tmp/mppi_obstacle_35m_scenario.svg",
    "MPPI 场景五：直线路径，前方 3.5m 障碍物", stats);
}

}  // namespace
