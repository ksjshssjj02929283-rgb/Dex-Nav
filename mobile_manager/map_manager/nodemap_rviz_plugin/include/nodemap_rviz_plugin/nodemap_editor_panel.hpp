#ifndef NODEMAP_RVIZ_PLUGIN__NODEMAP_EDITOR_PANEL_HPP_
#define NODEMAP_RVIZ_PLUGIN__NODEMAP_EDITOR_PANEL_HPP_

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include "rviz_common/panel.hpp"

namespace nodemap_rviz_plugin
{

class NodeMapEditorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit NodeMapEditorPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void chooseSourceMap();
  void chooseOutputDirectory();
  void generateNodeMaps();
  void loadExistingNodeMaps();
  void saveNodeMaps();
  void changeActiveLayer(int index);
  void activateMoveTool();
  void activateEdgeTool();
  void rebuildDefaultConnections();
  void refreshSummary();
  void showStatus(const QString & message);

private:
  struct LayerWidgets
  {
    QDoubleSpinBox * spacing{nullptr};
    QDoubleSpinBox * clearance{nullptr};
    QDoubleSpinBox * connection_radius{nullptr};
    QDoubleSpinBox * offset_x{nullptr};
    QDoubleSpinBox * offset_y{nullptr};
  };

  LayerWidgets createLayerWidgets(
    QFormLayout * form, const QString & prefix,
    double spacing, double clearance, double radius);
  void activateTool(const QString & class_id);
  void ensureEditorDisplays();
  void setSourcePath(const QString & path);

  QLineEdit * source_map_edit_{nullptr};
  QLineEdit * output_directory_edit_{nullptr};
  QPushButton * browse_map_button_{nullptr};
  QPushButton * browse_output_button_{nullptr};
  QPushButton * generate_button_{nullptr};
  QPushButton * load_existing_button_{nullptr};
  QPushButton * save_button_{nullptr};
  QCheckBox * unknown_obstacle_check_{nullptr};
  LayerWidgets global_widgets_;
  LayerWidgets local_widgets_;
  QComboBox * active_layer_combo_{nullptr};
  QCheckBox * manual_safety_check_{nullptr};
  QPushButton * move_node_button_{nullptr};
  QPushButton * edit_edge_button_{nullptr};
  QPushButton * rebuild_edges_button_{nullptr};
  QLabel * summary_label_{nullptr};
  QLabel * status_label_{nullptr};
};

}  // namespace nodemap_rviz_plugin

#endif  // NODEMAP_RVIZ_PLUGIN__NODEMAP_EDITOR_PANEL_HPP_
