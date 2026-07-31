#include "nodemap_rviz_plugin/editor_model.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <queue>
#include <system_error>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_map_server/map_io.hpp"
#include "nav2_map_server/node_map_io.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nodemap_rviz_plugin
{
namespace
{

constexpr float kSqrt2 = 1.41421356237F;

int gridIndex(int x, int y, int width)
{
  return y * width + x;
}

bool occupiedValue(int8_t value, bool unknown_as_obstacle)
{
  return value < 0 ? unknown_as_obstacle : value >= 50;
}

geometry_msgs::msg::Point markerPoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

std_msgs::msg::ColorRGBA markerColor(float red, float green, float blue, float alpha = 1.0F)
{
  std_msgs::msg::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
}

}  // namespace

EditorModel * EditorModel::instance()
{
  static EditorModel model;
  return &model;
}

EditorModel::EditorModel()
: QObject(nullptr)
{
}

void EditorModel::initializeRos(const rclcpp::Node::SharedPtr & node)
{
  node_ = node;
  if (!node_) {
    return;
  }
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  map_publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/nodemap_editor/map", qos);
  marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/nodemap_editor/markers", qos);
}

void EditorModel::clear()
{
  source_map_yaml_.clear();
  occupancy_ = nav_msgs::msg::OccupancyGrid();
  distance_map_ = nav2_map_server::DistanceMap();
  layers_ = {};
  default_edges_ = {};
  manual_added_edges_ = {};
  manual_removed_edges_ = {};
  source_loaded_ = false;
  graph_loaded_ = false;
  Q_EMIT modelChanged();
  publishMarkers();
}

bool EditorModel::loadSourceMap(const std::string & map_yaml, QString & error)
{
  const std::string expanded = nav2_map_server::expandUserPath(map_yaml);
  nav_msgs::msg::OccupancyGrid loaded;
  try {
    if (nav2_map_server::loadMapFromYaml(expanded, loaded) != nav2_map_server::LOAD_MAP_SUCCESS) {
      error = QString("读取原始地图失败：%1").arg(QString::fromStdString(expanded));
      return false;
    }
  } catch (const std::exception & exception) {
    error = QString("读取原始地图异常：%1").arg(exception.what());
    return false;
  }

  occupancy_ = std::move(loaded);
  occupancy_.header.frame_id = "map";
  source_map_yaml_ = expanded;
  source_loaded_ = true;
  graph_loaded_ = false;
  layers_ = {};
  default_edges_ = {};
  manual_added_edges_ = {};
  manual_removed_edges_ = {};
  publishMap();
  publishMarkers();
  Q_EMIT modelChanged();
  Q_EMIT statusMessage(
    QString("已导入地图：%1 × %2，分辨率 %3 m")
    .arg(occupancy_.info.width)
    .arg(occupancy_.info.height)
    .arg(occupancy_.info.resolution, 0, 'f', 3));
  return true;
}

nav2_map_server::DistanceMap EditorModel::computeDistanceMap(
  const BuildOptions & options) const
{
  nav2_map_server::DistanceMap result;
  result.width = static_cast<int>(occupancy_.info.width);
  result.height = static_cast<int>(occupancy_.info.height);
  result.resolution = occupancy_.info.resolution;
  result.origin_x = occupancy_.info.origin.position.x;
  result.origin_y = occupancy_.info.origin.position.y;
  result.max_distance_m = options.max_distance_m;

  const std::size_t cell_count =
    static_cast<std::size_t>(result.width * result.height);
  const float infinity = std::numeric_limits<float>::infinity();
  std::vector<float> distances(cell_count, infinity);

  struct QueueCell
  {
    float distance;
    int x;
    int y;
  };
  struct FurtherFirst
  {
    bool operator()(const QueueCell & lhs, const QueueCell & rhs) const
    {
      return lhs.distance > rhs.distance;
    }
  };
  std::priority_queue<QueueCell, std::vector<QueueCell>, FurtherFirst> queue;

  for (int y = 0; y < result.height; ++y) {
    for (int x = 0; x < result.width; ++x) {
      const int index = gridIndex(x, y, result.width);
      if (occupiedValue(
          occupancy_.data[static_cast<std::size_t>(index)],
          options.unknown_as_obstacle))
      {
        distances[static_cast<std::size_t>(index)] = 0.0F;
        queue.push({0.0F, x, y});
      }
    }
  }

  static const int delta_x[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int delta_y[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const float resolution = static_cast<float>(result.resolution);
  const float costs[8] = {
    resolution, resolution, resolution, resolution,
    resolution * kSqrt2, resolution * kSqrt2,
    resolution * kSqrt2, resolution * kSqrt2};

  while (!queue.empty()) {
    const QueueCell current = queue.top();
    queue.pop();
    const auto current_index = static_cast<std::size_t>(
      gridIndex(current.x, current.y, result.width));
    if (current.distance > distances[current_index]) {
      continue;
    }
    for (int direction = 0; direction < 8; ++direction) {
      const int next_x = current.x + delta_x[direction];
      const int next_y = current.y + delta_y[direction];
      if (
        next_x < 0 || next_y < 0 ||
        next_x >= result.width || next_y >= result.height)
      {
        continue;
      }
      const auto next_index = static_cast<std::size_t>(
        gridIndex(next_x, next_y, result.width));
      const float candidate = current.distance + costs[direction];
      if (candidate < distances[next_index]) {
        distances[next_index] = candidate;
        queue.push({candidate, next_x, next_y});
      }
    }
  }

  result.distance_m.resize(cell_count);
  for (std::size_t index = 0; index < cell_count; ++index) {
    result.distance_m[index] = std::isfinite(distances[index]) ?
      std::min(distances[index], static_cast<float>(options.max_distance_m)) :
      static_cast<float>(options.max_distance_m);
  }
  return result;
}

nav2_map_server::NodeMap EditorModel::buildSparseLayer(
  const std::string & layer_name,
  const LayerBuildOptions & options,
  const nav2_map_server::DistanceMap & distance_map) const
{
  nav2_map_server::NodeMap layer;
  layer.layer = layer_name;
  layer.source_map = "../map.yaml";
  layer.distance_map = "distance_map.yaml";
  layer.adjacency_mode = nav2_map_server::AdjacencyMode::EXPLICIT;
  layer.min_clearance_m = options.clearance_m;
  layer.node_spacing_m = options.spacing_m;
  layer.connection_radius_m = options.connection_radius_m;
  layer.resolution = occupancy_.info.resolution;
  layer.origin_x = occupancy_.info.origin.position.x;
  layer.origin_y = occupancy_.info.origin.position.y;
  layer.width = static_cast<int>(occupancy_.info.width);
  layer.height = static_cast<int>(occupancy_.info.height);

  const int step = std::max(
    1, static_cast<int>(std::round(options.spacing_m / layer.resolution)));
  auto firstCell = [step, resolution = layer.resolution](double offset_m) {
      const int offset_cells = static_cast<int>(std::round(offset_m / resolution));
      int first = (step / 2 + offset_cells) % step;
      if (first < 0) {
        first += step;
      }
      return first;
    };
  const int first_x = firstCell(options.offset_x_m);
  const int first_y = firstCell(options.offset_y_m);

  int next_id = 0;
  for (int gy = first_y; gy < layer.height; gy += step) {
    for (int gx = first_x; gx < layer.width; gx += step) {
      const auto index = static_cast<std::size_t>(gridIndex(gx, gy, layer.width));
      if (occupiedValue(occupancy_.data[index], last_build_options_.unknown_as_obstacle)) {
        continue;
      }
      if (distance_map.distance_m[index] < static_cast<float>(options.clearance_m)) {
        continue;
      }
      nav2_map_server::NodeMapNode node;
      node.id = next_id++;
      node.gx = gx;
      node.gy = gy;
      node.x = layer.origin_x + (gx + 0.5) * layer.resolution;
      node.y = layer.origin_y + (gy + 0.5) * layer.resolution;
      layer.nodes.push_back(node);
    }
  }
  layer.adjacency.resize(layer.nodes.size());
  return layer;
}

nav2_map_server::NodeMap EditorModel::buildRawLayer(const BuildOptions & options) const
{
  nav2_map_server::NodeMap layer;
  layer.layer = "raw";
  layer.source_map = "../map.yaml";
  layer.distance_map = "distance_map.yaml";
  layer.node_mask_image = "nodemap_raw_mask.pgm";
  layer.adjacency_mode = nav2_map_server::AdjacencyMode::GRID8;
  layer.min_clearance_m = 0.0;
  layer.node_spacing_m = occupancy_.info.resolution;
  layer.connection_radius_m = occupancy_.info.resolution * std::sqrt(2.0);
  layer.resolution = occupancy_.info.resolution;
  layer.origin_x = occupancy_.info.origin.position.x;
  layer.origin_y = occupancy_.info.origin.position.y;
  layer.width = static_cast<int>(occupancy_.info.width);
  layer.height = static_cast<int>(occupancy_.info.height);
  layer.grid_node_id.assign(
    static_cast<std::size_t>(layer.width * layer.height), -1);

  int next_id = 0;
  for (int gy = 0; gy < layer.height; ++gy) {
    for (int gx = 0; gx < layer.width; ++gx) {
      const auto index = static_cast<std::size_t>(gridIndex(gx, gy, layer.width));
      if (occupiedValue(occupancy_.data[index], options.unknown_as_obstacle)) {
        continue;
      }
      nav2_map_server::NodeMapNode node;
      node.id = next_id++;
      node.gx = gx;
      node.gy = gy;
      node.x = layer.origin_x + (gx + 0.5) * layer.resolution;
      node.y = layer.origin_y + (gy + 0.5) * layer.resolution;
      layer.grid_node_id[index] = node.id;
      layer.nodes.push_back(node);
    }
  }
  return layer;
}

bool EditorModel::generate(const BuildOptions & options, QString & error)
{
  if (!source_loaded_) {
    error = "请先导入原始 map.yaml";
    return false;
  }
  if (
    options.global.spacing_m <= 0.0 || options.local.spacing_m <= 0.0 ||
    options.global.connection_radius_m <= 0.0 ||
    options.local.connection_radius_m <= 0.0)
  {
    error = "节点间距和连接半径必须大于 0";
    return false;
  }

  last_build_options_ = options;
  distance_map_ = computeDistanceMap(options);
  layers_[0] = buildSparseLayer("global", options.global, distance_map_);
  layers_[1] = buildSparseLayer("local", options.local, distance_map_);
  layers_[2] = buildRawLayer(options);

  default_edges_[0] = computeDefaultEdges(layers_[0]);
  default_edges_[1] = computeDefaultEdges(layers_[1]);
  manual_added_edges_ = {};
  manual_removed_edges_ = {};
  writeEdgesToAdjacency(layers_[0], default_edges_[0]);
  writeEdgesToAdjacency(layers_[1], default_edges_[1]);
  graph_loaded_ = true;

  if (layers_[0].nodes.empty() || layers_[1].nodes.empty()) {
    error = "生成结果存在空层，请减小净空或调整间距/偏移";
    graph_loaded_ = false;
    return false;
  }

  publishAll();
  Q_EMIT modelChanged();
  Q_EMIT statusMessage(
    QString("节点图已生成：L1 %1 点 / L2 %2 点 / L3 %3 点")
    .arg(layers_[0].nodes.size())
    .arg(layers_[1].nodes.size())
    .arg(layers_[2].nodes.size()));
  return true;
}

bool EditorModel::loadExisting(const std::string & map_yaml, QString & error)
{
  if (!loadSourceMap(map_yaml, error)) {
    return false;
  }

  const std::string global_path = nav2_map_server::defaultGlobalNodeMapPath(source_map_yaml_);
  const std::string local_path = nav2_map_server::defaultLocalNodeMapPath(source_map_yaml_);
  const std::string raw_path = nav2_map_server::defaultRawNodeMapPath(source_map_yaml_);
  const std::string distance_path = nav2_map_server::defaultDistanceMapPath(source_map_yaml_);

  try {
    if (!nav2_map_server::loadNodeMap(global_path, layers_[0])) {
      error = QString("无法读取 L1：%1").arg(QString::fromStdString(global_path));
      return false;
    }
    if (!nav2_map_server::loadNodeMap(local_path, layers_[1])) {
      error = QString("无法读取 L2：%1").arg(QString::fromStdString(local_path));
      return false;
    }
    if (!nav2_map_server::loadNodeMap(raw_path, layers_[2])) {
      error = QString("无法读取 L3：%1").arg(QString::fromStdString(raw_path));
      return false;
    }
    if (!nav2_map_server::loadDistanceMap(distance_path, distance_map_)) {
      error = QString("无法读取距离图：%1").arg(QString::fromStdString(distance_path));
      return false;
    }
  } catch (const std::exception & exception) {
    error = QString("载入已有节点图异常：%1").arg(exception.what());
    return false;
  }

  last_build_options_.global.spacing_m = layers_[0].node_spacing_m;
  last_build_options_.global.clearance_m = layers_[0].min_clearance_m;
  last_build_options_.global.connection_radius_m = layers_[0].connection_radius_m;
  last_build_options_.local.spacing_m = layers_[1].node_spacing_m;
  last_build_options_.local.clearance_m = layers_[1].min_clearance_m;
  last_build_options_.local.connection_radius_m = layers_[1].connection_radius_m;
  inferLoadedOverrides(0);
  inferLoadedOverrides(1);
  graph_loaded_ = true;
  publishAll();
  Q_EMIT modelChanged();
  Q_EMIT statusMessage("已载入 preprocessed 目录中的三层节点图，可继续编辑");
  return true;
}

bool EditorModel::saveToDirectory(
  const std::string & output_directory, QString & error)
{
  if (!graph_loaded_) {
    error = "当前没有可保存的节点图";
    return false;
  }

  namespace fs = std::filesystem;
  const fs::path output_path(nav2_map_server::expandUserPath(output_directory));
  std::error_code filesystem_error;
  fs::create_directories(output_path, filesystem_error);
  if (filesystem_error) {
    error = QString("无法创建输出目录：%1").arg(filesystem_error.message().c_str());
    return false;
  }

  fs::path source_reference(source_map_yaml_);
  const fs::path relative_source =
    fs::relative(source_reference, output_path, filesystem_error);
  const std::string source_field = filesystem_error ?
    source_reference.string() : relative_source.string();

  for (auto & layer : layers_) {
    layer.source_map = source_field;
    layer.distance_map = "distance_map.yaml";
  }
  layers_[2].node_mask_image = "nodemap_raw_mask.pgm";

  const fs::path distance_yaml = output_path / "distance_map.yaml";
  const fs::path global_yaml = output_path / "nodemap_global.yaml";
  const fs::path local_yaml = output_path / "nodemap_local.yaml";
  const fs::path raw_yaml = output_path / "nodemap_raw.yaml";

  try {
    if (
      !nav2_map_server::saveDistanceMap(distance_map_, distance_yaml.string()) ||
      !nav2_map_server::saveNodeMap(layers_[0], global_yaml.string()) ||
      !nav2_map_server::saveNodeMap(layers_[1], local_yaml.string()) ||
      !nav2_map_server::saveNodeMap(layers_[2], raw_yaml.string()))
    {
      error = QString("写入节点图失败：%1").arg(QString::fromStdString(output_path.string()));
      return false;
    }
  } catch (const std::exception & exception) {
    error = QString("保存节点图异常：%1").arg(exception.what());
    return false;
  }

  Q_EMIT statusMessage(
    QString("已保存到 %1；Theta* 的 node_map_yaml 指向 nodemap_global.yaml")
    .arg(QString::fromStdString(output_path.string())));
  return true;
}

EditorModel::Edge EditorModel::normalizedEdge(int first_node_id, int second_node_id)
{
  return first_node_id < second_node_id ?
         Edge{first_node_id, second_node_id} :
         Edge{second_node_id, first_node_id};
}

int EditorModel::layerIndex(Layer layer)
{
  return static_cast<int>(layer);
}

bool EditorModel::hasActiveLayer() const
{
  return graph_loaded_ && !layers_[layerIndex(active_layer_)].nodes.empty();
}

bool EditorModel::activeLayerEditable() const
{
  return hasActiveLayer() && active_layer_ != Layer::RAW;
}

const nav2_map_server::NodeMap * EditorModel::activeNodeMap() const
{
  return hasActiveLayer() ? &layers_[layerIndex(active_layer_)] : nullptr;
}

void EditorModel::setActiveLayer(Layer layer)
{
  active_layer_ = layer;
  publishMarkers();
  Q_EMIT modelChanged();
}

bool EditorModel::worldToGrid(double x, double y, int & gx, int & gy) const
{
  if (!source_loaded_ || occupancy_.info.resolution <= 0.0) {
    return false;
  }
  gx = static_cast<int>(std::floor(
      (x - occupancy_.info.origin.position.x) / occupancy_.info.resolution));
  gy = static_cast<int>(std::floor(
      (y - occupancy_.info.origin.position.y) / occupancy_.info.resolution));
  return gx >= 0 && gy >= 0 &&
         gx < static_cast<int>(occupancy_.info.width) &&
         gy < static_cast<int>(occupancy_.info.height);
}

bool EditorModel::gridCellIsUsable(int gx, int gy, double clearance_m) const
{
  if (
    gx < 0 || gy < 0 ||
    gx >= static_cast<int>(occupancy_.info.width) ||
    gy >= static_cast<int>(occupancy_.info.height))
  {
    return false;
  }
  const auto index = static_cast<std::size_t>(
    gridIndex(gx, gy, static_cast<int>(occupancy_.info.width)));
  return !occupiedValue(
    occupancy_.data[index], last_build_options_.unknown_as_obstacle) &&
         index < distance_map_.distance_m.size() &&
         distance_map_.distance_m[index] >= static_cast<float>(clearance_m);
}

bool EditorModel::lineCellsAreSafe(
  int x0, int y0, int x1, int y1, double clearance_m) const
{
  int delta_x = std::abs(x1 - x0);
  int delta_y = std::abs(y1 - y0);
  const int step_x = x0 < x1 ? 1 : -1;
  const int step_y = y0 < y1 ? 1 : -1;
  int error = delta_x - delta_y;
  int x = x0;
  int y = y0;
  while (true) {
    if (!gridCellIsUsable(x, y, clearance_m)) {
      return false;
    }
    if (x == x1 && y == y1) {
      return true;
    }
    const int doubled_error = 2 * error;
    if (doubled_error > -delta_y) {
      error -= delta_y;
      x += step_x;
    }
    if (doubled_error < delta_x) {
      error += delta_x;
      y += step_y;
    }
  }
}

bool EditorModel::lineIsSafe(
  const nav2_map_server::NodeMap & layer,
  int first_node_id, int second_node_id) const
{
  if (
    first_node_id < 0 || second_node_id < 0 ||
    static_cast<std::size_t>(first_node_id) >= layer.nodes.size() ||
    static_cast<std::size_t>(second_node_id) >= layer.nodes.size())
  {
    return false;
  }
  const auto & first = layer.nodes[static_cast<std::size_t>(first_node_id)];
  const auto & second = layer.nodes[static_cast<std::size_t>(second_node_id)];
  return lineCellsAreSafe(
    first.gx, first.gy, second.gx, second.gy, layer.min_clearance_m);
}

EditorModel::EdgeSet EditorModel::computeDefaultEdges(
  const nav2_map_server::NodeMap & layer) const
{
  EdgeSet edges;
  const double radius_squared =
    layer.connection_radius_m * layer.connection_radius_m;
  const double bucket_size = std::max(
    layer.connection_radius_m, layer.resolution);

  auto bucketCoordinate = [bucket_size](double coordinate) {
      return static_cast<int>(std::floor(coordinate / bucket_size));
    };
  auto bucketKey = [](int bucket_x, int bucket_y) {
      return
        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(bucket_x)) << 32U) |
        static_cast<std::uint32_t>(bucket_y);
    };

  // 原离线构建器对全部节点两两检查，地图较大、L2 间距较小时会达到 O(N²)。
  // 编辑器按连接半径建立空间桶，只检查当前桶周围 3×3 个桶；连接判定和
  // Bresenham 净空检查完全不变，但生成规模近似 O(N·局部邻居数)。
  std::unordered_map<std::uint64_t, std::vector<int>> buckets;
  buckets.reserve(layer.nodes.size());
  for (std::size_t current = 0; current < layer.nodes.size(); ++current) {
    const auto & current_node = layer.nodes[current];
    const int current_bucket_x = bucketCoordinate(current_node.x);
    const int current_bucket_y = bucketCoordinate(current_node.y);
    for (int offset_y = -1; offset_y <= 1; ++offset_y) {
      for (int offset_x = -1; offset_x <= 1; ++offset_x) {
        const auto found = buckets.find(
          bucketKey(current_bucket_x + offset_x, current_bucket_y + offset_y));
        if (found == buckets.end()) {
          continue;
        }
        for (const int previous : found->second) {
          const auto & previous_node =
            layer.nodes[static_cast<std::size_t>(previous)];
          const double dx = current_node.x - previous_node.x;
          const double dy = current_node.y - previous_node.y;
          if (dx * dx + dy * dy > radius_squared) {
            continue;
          }
          if (lineIsSafe(layer, previous, static_cast<int>(current))) {
            edges.insert({previous, static_cast<int>(current)});
          }
        }
      }
    }
    buckets[bucketKey(current_bucket_x, current_bucket_y)].push_back(
      static_cast<int>(current));
  }
  return edges;
}

EditorModel::EdgeSet EditorModel::adjacencyEdges(
  const nav2_map_server::NodeMap & layer) const
{
  EdgeSet edges;
  for (
    std::size_t first = 0;
    first < layer.adjacency.size() && first < layer.nodes.size(); ++first)
  {
    for (const int second : layer.adjacency[first]) {
      if (
        second >= 0 && static_cast<std::size_t>(second) < layer.nodes.size() &&
        second != static_cast<int>(first))
      {
        edges.insert(normalizedEdge(static_cast<int>(first), second));
      }
    }
  }
  return edges;
}

void EditorModel::writeEdgesToAdjacency(
  nav2_map_server::NodeMap & layer, const EdgeSet & edges) const
{
  layer.adjacency.assign(layer.nodes.size(), {});
  for (const auto & edge : edges) {
    if (
      edge.first < 0 || edge.second < 0 ||
      static_cast<std::size_t>(edge.second) >= layer.nodes.size())
    {
      continue;
    }
    layer.adjacency[static_cast<std::size_t>(edge.first)].push_back(edge.second);
    layer.adjacency[static_cast<std::size_t>(edge.second)].push_back(edge.first);
  }
  for (auto & neighbors : layer.adjacency) {
    std::sort(neighbors.begin(), neighbors.end());
  }
}

void EditorModel::rebuildLayerConnections(int layer_index)
{
  default_edges_[layer_index] = computeDefaultEdges(layers_[layer_index]);
  EdgeSet final_edges = default_edges_[layer_index];
  for (const auto & edge : manual_removed_edges_[layer_index]) {
    final_edges.erase(edge);
  }
  final_edges.insert(
    manual_added_edges_[layer_index].begin(),
    manual_added_edges_[layer_index].end());
  writeEdgesToAdjacency(layers_[layer_index], final_edges);
}

void EditorModel::inferLoadedOverrides(int layer_index)
{
  const EdgeSet loaded_edges = adjacencyEdges(layers_[layer_index]);
  default_edges_[layer_index] = computeDefaultEdges(layers_[layer_index]);
  manual_added_edges_[layer_index].clear();
  manual_removed_edges_[layer_index].clear();
  std::set_difference(
    loaded_edges.begin(), loaded_edges.end(),
    default_edges_[layer_index].begin(), default_edges_[layer_index].end(),
    std::inserter(manual_added_edges_[layer_index], manual_added_edges_[layer_index].end()));
  std::set_difference(
    default_edges_[layer_index].begin(), default_edges_[layer_index].end(),
    loaded_edges.begin(), loaded_edges.end(),
    std::inserter(
      manual_removed_edges_[layer_index], manual_removed_edges_[layer_index].end()));
}

int EditorModel::nearestActiveNode(
  double x, double y, double max_distance_m) const
{
  const auto * layer = activeNodeMap();
  if (!layer) {
    return -1;
  }
  const double maximum_squared = max_distance_m * max_distance_m;
  double best_squared = maximum_squared;
  int best_id = -1;
  for (const auto & node : layer->nodes) {
    const double dx = node.x - x;
    const double dy = node.y - y;
    const double squared = dx * dx + dy * dy;
    if (squared <= best_squared) {
      best_squared = squared;
      best_id = node.id;
    }
  }
  return best_id;
}

bool EditorModel::moveActiveNode(
  int node_id, double x, double y, QString & error)
{
  if (!activeLayerEditable()) {
    error = "L3 raw 层只读；请选择 L1 或 L2";
    return false;
  }
  const int index = layerIndex(active_layer_);
  auto & layer = layers_[index];
  if (node_id < 0 || static_cast<std::size_t>(node_id) >= layer.nodes.size()) {
    error = "待移动节点不存在";
    return false;
  }

  int gx = 0;
  int gy = 0;
  if (!worldToGrid(x, y, gx, gy)) {
    error = "目标位置超出原始地图范围";
    return false;
  }
  if (!gridCellIsUsable(gx, gy, layer.min_clearance_m)) {
    error = "目标位置占用或不满足本层最小净空";
    return false;
  }
  for (const auto & node : layer.nodes) {
    if (node.id != node_id && node.gx == gx && node.gy == gy) {
      error = QString("目标栅格已经存在节点 %1").arg(node.id);
      return false;
    }
  }

  auto & node = layer.nodes[static_cast<std::size_t>(node_id)];
  node.x = x;
  node.y = y;
  node.gx = gx;
  node.gy = gy;
  rebuildLayerConnections(index);
  publishMarkers();
  Q_EMIT modelChanged();
  Q_EMIT statusMessage(
    QString("节点 %1 已移动到 (%2, %3)，默认安全连接已自动重算")
    .arg(node_id).arg(x, 0, 'f', 3).arg(y, 0, 'f', 3));
  return true;
}

bool EditorModel::activeEdgeExists(int first_node_id, int second_node_id) const
{
  const auto * layer = activeNodeMap();
  if (!layer) {
    return false;
  }
  return adjacencyEdges(*layer).count(normalizedEdge(first_node_id, second_node_id)) != 0;
}

bool EditorModel::activeEdgeIsSafe(int first_node_id, int second_node_id) const
{
  const auto * layer = activeNodeMap();
  return layer && lineIsSafe(*layer, first_node_id, second_node_id);
}

bool EditorModel::activeEdgeIsManual(int first_node_id, int second_node_id) const
{
  if (!activeLayerEditable()) {
    return false;
  }
  const int index = layerIndex(active_layer_);
  return manual_added_edges_[index].count(
    normalizedEdge(first_node_id, second_node_id)) != 0;
}

bool EditorModel::toggleActiveEdge(
  int first_node_id, int second_node_id, bool enforce_safety, QString & error)
{
  if (!activeLayerEditable()) {
    error = "L3 raw 层连接关系由 GRID8 固定生成，不能手工编辑";
    return false;
  }
  if (first_node_id == second_node_id) {
    error = "不能连接节点自身";
    return false;
  }

  const int index = layerIndex(active_layer_);
  auto & layer = layers_[index];
  if (
    first_node_id < 0 || second_node_id < 0 ||
    static_cast<std::size_t>(first_node_id) >= layer.nodes.size() ||
    static_cast<std::size_t>(second_node_id) >= layer.nodes.size())
  {
    error = "连接端点不存在";
    return false;
  }

  const Edge edge = normalizedEdge(first_node_id, second_node_id);
  EdgeSet final_edges = adjacencyEdges(layer);
  if (final_edges.erase(edge) != 0) {
    if (manual_added_edges_[index].erase(edge) == 0) {
      manual_removed_edges_[index].insert(edge);
    }
    writeEdgesToAdjacency(layer, final_edges);
    Q_EMIT statusMessage(
      QString("已人工删除连接：%1 ↔ %2").arg(edge.first).arg(edge.second));
  } else {
    if (enforce_safety && !lineIsSafe(layer, edge.first, edge.second)) {
      error = "该连线穿过障碍物或不满足本层净空；取消“手工连边安全校验”后可强制添加";
      return false;
    }
    if (default_edges_[index].count(edge) != 0) {
      manual_removed_edges_[index].erase(edge);
    } else {
      manual_added_edges_[index].insert(edge);
    }
    final_edges.insert(edge);
    writeEdgesToAdjacency(layer, final_edges);
    Q_EMIT statusMessage(
      QString("已人工添加连接：%1 ↔ %2%3")
      .arg(edge.first).arg(edge.second)
      .arg(lineIsSafe(layer, edge.first, edge.second) ? "" : "（强制，当前不安全）"));
  }

  publishMarkers();
  Q_EMIT modelChanged();
  return true;
}

bool EditorModel::rebuildActiveDefaultConnections(QString & error)
{
  if (!activeLayerEditable()) {
    error = "L3 raw 层不使用显式连接表";
    return false;
  }
  const int index = layerIndex(active_layer_);
  manual_added_edges_[index].clear();
  manual_removed_edges_[index].clear();
  rebuildLayerConnections(index);
  publishMarkers();
  Q_EMIT modelChanged();
  Q_EMIT statusMessage("已清除本层人工连接修改，并按障碍物/净空重新生成默认连接");
  return true;
}

std::size_t EditorModel::activeNodeCount() const
{
  const auto * layer = activeNodeMap();
  return layer ? layer->nodes.size() : 0;
}

std::size_t EditorModel::activeEdgeCount() const
{
  const auto * layer = activeNodeMap();
  if (!layer) {
    return 0;
  }
  if (layer->adjacency_mode == nav2_map_server::AdjacencyMode::GRID8) {
    std::size_t count = 0;
    static const int forward_moves[4][2] = {{1, 0}, {0, 1}, {1, 1}, {-1, 1}};
    for (const auto & node : layer->nodes) {
      for (const auto & move : forward_moves) {
        const int gx = node.gx + move[0];
        const int gy = node.gy + move[1];
        if (
          gx >= 0 && gy >= 0 && gx < layer->width && gy < layer->height &&
          layer->grid_node_id[
            static_cast<std::size_t>(gridIndex(gx, gy, layer->width))] >= 0)
        {
          ++count;
        }
      }
    }
    return count;
  }
  return adjacencyEdges(*layer).size();
}

std::size_t EditorModel::activeUnsafeEdgeCount() const
{
  if (!activeLayerEditable()) {
    return 0;
  }
  const auto & layer = layers_[layerIndex(active_layer_)];
  std::size_t count = 0;
  for (const auto & edge : adjacencyEdges(layer)) {
    if (!lineIsSafe(layer, edge.first, edge.second)) {
      ++count;
    }
  }
  return count;
}

double EditorModel::activeNodeSpacing() const
{
  const auto * layer = activeNodeMap();
  return layer ? layer->node_spacing_m : 0.0;
}

void EditorModel::publishMap()
{
  if (!map_publisher_ || !source_loaded_) {
    return;
  }
  occupancy_.header.frame_id = "map";
  occupancy_.header.stamp = node_ ? node_->now() : rclcpp::Time(0);
  map_publisher_->publish(occupancy_);
}

void EditorModel::publishMarkers()
{
  if (!marker_publisher_) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = "map";
  clear_marker.header.stamp = rclcpp::Time(0);
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear_marker);

  const auto * layer = activeNodeMap();
  if (!layer) {
    marker_publisher_->publish(array);
    return;
  }

  const int index = layerIndex(active_layer_);
  const std_msgs::msg::ColorRGBA layer_color =
    active_layer_ == Layer::GLOBAL ? markerColor(0.10F, 0.48F, 0.95F) :
    active_layer_ == Layer::LOCAL ? markerColor(0.18F, 0.78F, 0.30F) :
    markerColor(0.95F, 0.42F, 0.08F);

  visualization_msgs::msg::Marker nodes;
  nodes.header.frame_id = "map";
  nodes.header.stamp = rclcpp::Time(0);
  nodes.ns = "nodemap_editor_nodes";
  nodes.id = 1;
  nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  nodes.action = visualization_msgs::msg::Marker::ADD;
  nodes.pose.orientation.w = 1.0;
  const double node_scale = active_layer_ == Layer::RAW ?
    std::max(0.025, layer->resolution * 0.55) :
    std::clamp(layer->node_spacing_m * 0.22, 0.07, 0.22);
  nodes.scale.x = node_scale;
  nodes.scale.y = node_scale;
  nodes.scale.z = node_scale;
  nodes.color = layer_color;
  nodes.frame_locked = true;
  nodes.points.reserve(layer->nodes.size());
  for (const auto & node : layer->nodes) {
    nodes.points.push_back(markerPoint(node.x, node.y, 0.12));
  }
  array.markers.push_back(std::move(nodes));

  // L3 节点通常达到数万，绘制 GRID8 全部边会显著拖慢 RViz；L3 仍可通过
  // 橙色节点完整查看，只有可编辑的 L1/L2 绘制显式连接。
  if (active_layer_ != Layer::RAW) {
    visualization_msgs::msg::Marker edges;
    edges.header.frame_id = "map";
    edges.header.stamp = rclcpp::Time(0);
    edges.ns = "nodemap_editor_edges";
    edges.id = 2;
    edges.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges.action = visualization_msgs::msg::Marker::ADD;
    edges.pose.orientation.w = 1.0;
    edges.scale.x = std::clamp(layer->node_spacing_m * 0.035, 0.015, 0.045);
    edges.color.a = 1.0F;
    edges.frame_locked = true;
    for (const auto & edge : adjacencyEdges(*layer)) {
      const auto & first = layer->nodes[static_cast<std::size_t>(edge.first)];
      const auto & second = layer->nodes[static_cast<std::size_t>(edge.second)];
      edges.points.push_back(markerPoint(first.x, first.y, 0.08));
      edges.points.push_back(markerPoint(second.x, second.y, 0.08));
      const bool safe = lineIsSafe(*layer, edge.first, edge.second);
      const bool manual = manual_added_edges_[index].count(edge) != 0;
      const auto color = !safe ? markerColor(1.0F, 0.05F, 0.05F) :
        manual ? markerColor(1.0F, 0.86F, 0.05F) :
        markerColor(layer_color.r, layer_color.g, layer_color.b, 0.68F);
      edges.colors.push_back(color);
      edges.colors.push_back(color);
    }
    array.markers.push_back(std::move(edges));
  }

  marker_publisher_->publish(array);
}

void EditorModel::publishAll()
{
  publishMap();
  publishMarkers();
}

}  // namespace nodemap_rviz_plugin
