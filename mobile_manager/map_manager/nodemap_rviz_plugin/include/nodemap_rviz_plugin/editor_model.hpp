#ifndef NODEMAP_RVIZ_PLUGIN__EDITOR_MODEL_HPP_
#define NODEMAP_RVIZ_PLUGIN__EDITOR_MODEL_HPP_

#include <QObject>
#include <QString>

#include <array>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include "nav2_map_server/node_map_types.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nodemap_rviz_plugin
{

struct LayerBuildOptions
{
  double spacing_m{0.60};
  double clearance_m{0.40};
  double connection_radius_m{0.90};
  // 相对于原有“半个间距起铺”规则的附加偏移，可用于错开两层规则网格。
  double offset_x_m{0.0};
  double offset_y_m{0.0};
};

struct BuildOptions
{
  LayerBuildOptions global;
  LayerBuildOptions local{0.40, 0.30, 0.70, 0.0, 0.0};
  bool unknown_as_obstacle{true};
  double max_distance_m{2.0};
};

/**
 * @brief RViz 面板和交互工具共享的节点图编辑模型。
 *
 * 模型只读写 nav2_map_server 已有的 NodeMap YAML 格式，不修改 map_server、
 * Theta* 或正在运行的地图服务。所有编辑先保存在内存，按“保存”后才落盘。
 */
class EditorModel : public QObject
{
  Q_OBJECT

public:
  enum class Layer
  {
    GLOBAL = 0,
    LOCAL = 1,
    RAW = 2
  };

  static EditorModel * instance();

  void initializeRos(const rclcpp::Node::SharedPtr & node);

  bool loadSourceMap(const std::string & map_yaml, QString & error);
  bool generate(const BuildOptions & options, QString & error);
  bool loadExisting(const std::string & map_yaml, QString & error);
  bool saveToDirectory(const std::string & output_directory, QString & error);

  void setActiveLayer(Layer layer);
  Layer activeLayer() const {return active_layer_;}
  bool hasSourceMap() const {return source_loaded_;}
  bool hasActiveLayer() const;
  bool activeLayerEditable() const;
  const std::string & sourceMapYaml() const {return source_map_yaml_;}
  const nav2_map_server::NodeMap * activeNodeMap() const;

  int nearestActiveNode(double x, double y, double max_distance_m) const;
  bool moveActiveNode(int node_id, double x, double y, QString & error);
  bool toggleActiveEdge(
    int first_node_id, int second_node_id, bool enforce_safety, QString & error);
  bool rebuildActiveDefaultConnections(QString & error);

  bool activeEdgeExists(int first_node_id, int second_node_id) const;
  bool activeEdgeIsSafe(int first_node_id, int second_node_id) const;
  bool activeEdgeIsManual(int first_node_id, int second_node_id) const;
  void setManualEdgeSafetyCheck(bool enabled) {manual_edge_safety_check_ = enabled;}
  bool manualEdgeSafetyCheck() const {return manual_edge_safety_check_;}

  std::size_t activeNodeCount() const;
  std::size_t activeEdgeCount() const;
  std::size_t activeUnsafeEdgeCount() const;
  double activeNodeSpacing() const;

  void publishAll();
  void clear();

Q_SIGNALS:
  void modelChanged();
  void statusMessage(const QString & message);

private:
  using Edge = std::pair<int, int>;
  using EdgeSet = std::set<Edge>;

  EditorModel();

  static Edge normalizedEdge(int first_node_id, int second_node_id);
  static int layerIndex(Layer layer);

  nav2_map_server::DistanceMap computeDistanceMap(const BuildOptions & options) const;
  nav2_map_server::NodeMap buildSparseLayer(
    const std::string & layer_name,
    const LayerBuildOptions & options,
    const nav2_map_server::DistanceMap & distance_map) const;
  nav2_map_server::NodeMap buildRawLayer(const BuildOptions & options) const;

  EdgeSet computeDefaultEdges(const nav2_map_server::NodeMap & layer) const;
  EdgeSet adjacencyEdges(const nav2_map_server::NodeMap & layer) const;
  void writeEdgesToAdjacency(nav2_map_server::NodeMap & layer, const EdgeSet & edges) const;
  void rebuildLayerConnections(int layer_index);
  void inferLoadedOverrides(int layer_index);

  bool worldToGrid(double x, double y, int & gx, int & gy) const;
  bool gridCellIsUsable(int gx, int gy, double clearance_m) const;
  bool lineIsSafe(
    const nav2_map_server::NodeMap & layer,
    int first_node_id, int second_node_id) const;
  bool lineCellsAreSafe(
    int x0, int y0, int x1, int y1, double clearance_m) const;

  void publishMap();
  void publishMarkers();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

  std::string source_map_yaml_;
  nav_msgs::msg::OccupancyGrid occupancy_;
  nav2_map_server::DistanceMap distance_map_;
  std::array<nav2_map_server::NodeMap, 3> layers_;

  // 默认边和人工增删分别记录。拖动节点后只重算默认边，再叠加人工修改，
  // 从而不会因为移动另一个节点而丢失已经手工补上的连接关系。
  std::array<EdgeSet, 2> default_edges_;
  std::array<EdgeSet, 2> manual_added_edges_;
  std::array<EdgeSet, 2> manual_removed_edges_;

  Layer active_layer_{Layer::GLOBAL};
  BuildOptions last_build_options_;
  bool source_loaded_{false};
  bool graph_loaded_{false};
  bool manual_edge_safety_check_{true};
};

}  // namespace nodemap_rviz_plugin

#endif  // NODEMAP_RVIZ_PLUGIN__EDITOR_MODEL_HPP_
