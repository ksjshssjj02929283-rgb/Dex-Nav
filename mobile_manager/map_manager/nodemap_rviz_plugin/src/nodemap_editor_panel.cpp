#include "nodemap_rviz_plugin/nodemap_editor_panel.hpp"

#include <QApplication>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QScrollArea>
#include <QSizePolicy>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <filesystem>

#include "nodemap_rviz_plugin/editor_model.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/display_group.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/tool_manager.hpp"
#include "rviz_common/visualization_manager.hpp"

namespace nodemap_rviz_plugin
{
namespace
{

constexpr char kMoveNodeToolId[] = "nodemap_rviz_plugin/MoveNodeTool";
constexpr char kEditEdgeToolId[] = "nodemap_rviz_plugin/EditEdgeTool";

QDoubleSpinBox * makeDistanceSpin(
  double minimum, double maximum, double value, const QString & suffix = " m")
{
  auto * spin = new QDoubleSpinBox();
  spin->setDecimals(3);
  spin->setRange(minimum, maximum);
  spin->setSingleStep(0.05);
  spin->setValue(value);
  spin->setSuffix(suffix);
  return spin;
}

}  // namespace

NodeMapEditorPanel::NodeMapEditorPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  source_map_edit_ = new QLineEdit();
  source_map_edit_->setPlaceholderText("选择原始 map.yaml");
  browse_map_button_ = new QPushButton("导入地图…");
  auto * map_row = new QHBoxLayout();
  map_row->addWidget(source_map_edit_, 1);
  map_row->addWidget(browse_map_button_);

  output_directory_edit_ = new QLineEdit();
  output_directory_edit_->setPlaceholderText("<地图目录>/preprocessed");
  browse_output_button_ = new QPushButton("输出目录…");
  auto * output_row = new QHBoxLayout();
  output_row->addWidget(output_directory_edit_, 1);
  output_row->addWidget(browse_output_button_);

  auto * file_form = new QFormLayout();
  file_form->addRow("原始地图:", map_row);
  file_form->addRow("保存目录:", output_row);
  auto * file_box = new QGroupBox("1. 地图文件");
  file_box->setLayout(file_form);

  auto * global_form = new QFormLayout();
  global_widgets_ = createLayerWidgets(
    global_form, "L1", 0.60, 0.40, 0.90);
  auto * global_box = new QGroupBox("L1 全局层");
  global_box->setLayout(global_form);

  auto * local_form = new QFormLayout();
  local_widgets_ = createLayerWidgets(
    local_form, "L2", 0.40, 0.30, 0.70);
  auto * local_box = new QGroupBox("L2 局部层");
  local_box->setLayout(local_form);

  unknown_obstacle_check_ = new QCheckBox("未知区域按障碍物处理");
  unknown_obstacle_check_->setChecked(true);
  unknown_obstacle_check_->setToolTip(
    "建议保持勾选，防止规则节点和默认连线进入未建图区域。");

  generate_button_ = new QPushButton("生成三层节点图");
  load_existing_button_ = new QPushButton("载入已有 preprocessed");
  save_button_ = new QPushButton("保存供 Theta* 使用");
  auto * generation_row = new QHBoxLayout();
  generation_row->addWidget(generate_button_);
  generation_row->addWidget(load_existing_button_);
  generation_row->addWidget(save_button_);

  auto * generation_layout = new QVBoxLayout();
  generation_layout->addWidget(global_box);
  generation_layout->addWidget(local_box);
  generation_layout->addWidget(unknown_obstacle_check_);
  generation_layout->addLayout(generation_row);
  auto * generation_box = new QGroupBox("2. 均匀生成");
  generation_box->setLayout(generation_layout);

  active_layer_combo_ = new QComboBox();
  active_layer_combo_->addItem("L1 global（可编辑）");
  active_layer_combo_->addItem("L2 local（可编辑）");
  active_layer_combo_->addItem("L3 raw（只读预览）");

  move_node_button_ = new QPushButton("拖动节点 [M]");
  edit_edge_button_ = new QPushButton("添加/删除连接 [E]");
  rebuild_edges_button_ = new QPushButton("恢复本层默认连接");
  manual_safety_check_ = new QCheckBox("手工连边也执行障碍物/净空校验");
  manual_safety_check_->setChecked(true);
  manual_safety_check_->setToolTip(
    "取消后可以强制添加不安全边；这种边会显示为红色，保存前请谨慎确认。");

  auto * edit_button_row = new QHBoxLayout();
  edit_button_row->addWidget(move_node_button_);
  edit_button_row->addWidget(edit_edge_button_);
  auto * edit_form = new QFormLayout();
  edit_form->addRow("当前显示层:", active_layer_combo_);
  edit_form->addRow(edit_button_row);
  edit_form->addRow(rebuild_edges_button_);
  edit_form->addRow(manual_safety_check_);
  auto * edit_box = new QGroupBox("3. RViz 可视化编辑");
  edit_box->setLayout(edit_form);

  summary_label_ = new QLabel("尚未生成节点图");
  summary_label_->setWordWrap(true);
  status_label_ = new QLabel(
    "流程：导入 map.yaml → 设置两层参数 → 生成 → 拖动/连边 → 保存");
  status_label_->setWordWrap(true);
  status_label_->setStyleSheet(
    "QLabel { background: #263238; color: #ECEFF1; padding: 6px; }");

  // 地图文件与两层生成参数内容较多。RViz 的 Panel 停靠后没有普通窗口的
  // 最大化按钮，若直接把所有控件铺在根布局中，小屏幕或横向停靠时底部按钮
  // 会被挤出屏幕。这里让参数区独立滚动，并把高频编辑区固定在面板底部，
  // 保证“拖动节点/编辑连接”在任意面板高度下都能直接操作。
  auto * parameter_widget = new QWidget();
  auto * parameter_layout = new QVBoxLayout(parameter_widget);
  parameter_layout->setContentsMargins(0, 0, 0, 0);
  parameter_layout->addWidget(file_box);
  parameter_layout->addWidget(generation_box);
  parameter_layout->addStretch(1);

  auto * parameter_scroll = new QScrollArea();
  parameter_scroll->setWidgetResizable(true);
  parameter_scroll->setFrameShape(QFrame::NoFrame);
  parameter_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  parameter_scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  parameter_scroll->setSizePolicy(
    QSizePolicy::Expanding, QSizePolicy::Expanding);
  parameter_scroll->setWidget(parameter_widget);

  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(4, 4, 4, 4);
  root->addWidget(parameter_scroll, 1);
  root->addWidget(edit_box, 0);
  root->addWidget(summary_label_, 0);
  root->addWidget(status_label_, 0);

  // 不给 Panel 设置大于屏幕的最小尺寸，让 RViz 可以自由停靠和缩放；
  // 参数超出可用区域时由上面的滚动条承载。
  setMinimumSize(0, 0);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

  connect(
    browse_map_button_, &QPushButton::clicked,
    this, &NodeMapEditorPanel::chooseSourceMap);
  connect(
    browse_output_button_, &QPushButton::clicked,
    this, &NodeMapEditorPanel::chooseOutputDirectory);
  connect(
    generate_button_, &QPushButton::clicked,
    this, &NodeMapEditorPanel::generateNodeMaps);
  connect(
    load_existing_button_, &QPushButton::clicked,
    this, &NodeMapEditorPanel::loadExistingNodeMaps);
  connect(
    save_button_, &QPushButton::clicked,
    this, &NodeMapEditorPanel::saveNodeMaps);
  connect(
    active_layer_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
    this, &NodeMapEditorPanel::changeActiveLayer);
  connect(
    move_node_button_, &QPushButton::clicked,
    this, &NodeMapEditorPanel::activateMoveTool);
  connect(
    edit_edge_button_, &QPushButton::clicked,
    this, &NodeMapEditorPanel::activateEdgeTool);
  connect(
    rebuild_edges_button_, &QPushButton::clicked,
    this, &NodeMapEditorPanel::rebuildDefaultConnections);
  connect(
    manual_safety_check_, &QCheckBox::toggled,
    EditorModel::instance(), &EditorModel::setManualEdgeSafetyCheck);
  connect(
    EditorModel::instance(), &EditorModel::modelChanged,
    this, &NodeMapEditorPanel::refreshSummary);
  connect(
    EditorModel::instance(), &EditorModel::statusMessage,
    this, &NodeMapEditorPanel::showStatus);
}

NodeMapEditorPanel::LayerWidgets NodeMapEditorPanel::createLayerWidgets(
  QFormLayout * form, const QString & prefix,
  double spacing, double clearance, double radius)
{
  LayerWidgets widgets;
  widgets.spacing = makeDistanceSpin(0.05, 10.0, spacing);
  widgets.clearance = makeDistanceSpin(0.0, 5.0, clearance);
  widgets.connection_radius = makeDistanceSpin(0.05, 20.0, radius);
  widgets.offset_x = makeDistanceSpin(-10.0, 10.0, 0.0);
  widgets.offset_y = makeDistanceSpin(-10.0, 10.0, 0.0);
  widgets.offset_x->setToolTip(
    "在默认半间距起铺位置上增加 X 偏移；最终按地图分辨率取整并对间距循环。");
  widgets.offset_y->setToolTip(
    "在默认半间距起铺位置上增加 Y 偏移；可让 L1/L2 节点错开。");
  form->addRow(prefix + " 节点间距:", widgets.spacing);
  form->addRow(prefix + " 最小净空:", widgets.clearance);
  form->addRow(prefix + " 默认连接半径:", widgets.connection_radius);
  form->addRow(prefix + " X 起铺偏移:", widgets.offset_x);
  form->addRow(prefix + " Y 起铺偏移:", widgets.offset_y);
  return widgets;
}

void NodeMapEditorPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    showStatus("无法取得 RViz ROS 节点");
    return;
  }
  EditorModel::instance()->initializeRos(abstraction->get_raw_node());
  ensureEditorDisplays();
  EditorModel::instance()->publishAll();
}

void NodeMapEditorPanel::ensureEditorDisplays()
{
  // RViz 的 DisplayContext 实际由 VisualizationManager 实现；使用
  // dynamic_cast 保持插件对测试/其他 DisplayContext 实现的兼容性。
  auto * manager = dynamic_cast<rviz_common::VisualizationManager *>(
    getDisplayContext());
  auto * root = manager ? manager->getRootDisplayGroup() : nullptr;
  if (!manager || !root) {
    return;
  }

  auto findDisplay = [root](const QString & name) -> rviz_common::Display * {
      for (int index = 0; index < root->numDisplays(); ++index) {
        auto * display = root->getDisplayAt(index);
        if (display && display->getName() == name) {
          return display;
        }
      }
      return nullptr;
    };

  auto * map_display = findDisplay("NodeMap Editor Source Map");
  if (!map_display) {
    map_display = manager->createDisplay(
      "rviz_default_plugins/Map", "NodeMap Editor Source Map", true);
  }
  if (map_display) {
    map_display->subProp("Topic")->setValue("/nodemap_editor/map");
  }

  auto * marker_display = findDisplay("NodeMap Editor Graph");
  if (!marker_display) {
    marker_display = manager->createDisplay(
      "rviz_default_plugins/MarkerArray", "NodeMap Editor Graph", true);
  }
  if (marker_display) {
    marker_display->subProp("Topic")->setValue("/nodemap_editor/markers");
  }
}

void NodeMapEditorPanel::setSourcePath(const QString & path)
{
  source_map_edit_->setText(path);
  if (path.isEmpty()) {
    return;
  }
  const std::filesystem::path map_path(path.toStdString());
  output_directory_edit_->setText(
    QString::fromStdString((map_path.parent_path() / "preprocessed").string()));
}

void NodeMapEditorPanel::chooseSourceMap()
{
  const QString selected = QFileDialog::getOpenFileName(
    this, "选择原始地图 YAML", source_map_edit_->text(),
    "ROS map YAML (*.yaml *.yml)");
  if (selected.isEmpty()) {
    return;
  }
  setSourcePath(selected);
  QString error;
  if (!EditorModel::instance()->loadSourceMap(selected.toStdString(), error)) {
    QMessageBox::critical(this, "导入地图失败", error);
  }
}

void NodeMapEditorPanel::chooseOutputDirectory()
{
  const QString selected = QFileDialog::getExistingDirectory(
    this, "选择节点图输出目录", output_directory_edit_->text());
  if (!selected.isEmpty()) {
    output_directory_edit_->setText(selected);
  }
}

void NodeMapEditorPanel::generateNodeMaps()
{
  QString error;
  const QString source = source_map_edit_->text().trimmed();
  if (source.isEmpty()) {
    QMessageBox::warning(this, "缺少地图", "请先选择原始 map.yaml");
    return;
  }
  if (
    !EditorModel::instance()->hasSourceMap() ||
    EditorModel::instance()->sourceMapYaml() != source.toStdString())
  {
    if (!EditorModel::instance()->loadSourceMap(source.toStdString(), error)) {
      QMessageBox::critical(this, "导入地图失败", error);
      return;
    }
  }

  BuildOptions options;
  options.global = {
    global_widgets_.spacing->value(),
    global_widgets_.clearance->value(),
    global_widgets_.connection_radius->value(),
    global_widgets_.offset_x->value(),
    global_widgets_.offset_y->value()};
  options.local = {
    local_widgets_.spacing->value(),
    local_widgets_.clearance->value(),
    local_widgets_.connection_radius->value(),
    local_widgets_.offset_x->value(),
    local_widgets_.offset_y->value()};
  options.unknown_as_obstacle = unknown_obstacle_check_->isChecked();

  QApplication::setOverrideCursor(Qt::WaitCursor);
  const bool generated = EditorModel::instance()->generate(options, error);
  QApplication::restoreOverrideCursor();
  if (!generated) {
    QMessageBox::critical(this, "生成节点图失败", error);
  }
}

void NodeMapEditorPanel::loadExistingNodeMaps()
{
  const QString source = source_map_edit_->text().trimmed();
  if (source.isEmpty()) {
    QMessageBox::warning(this, "缺少地图", "请先选择原始 map.yaml");
    return;
  }
  QString error;
  QApplication::setOverrideCursor(Qt::WaitCursor);
  const bool loaded = EditorModel::instance()->loadExisting(source.toStdString(), error);
  QApplication::restoreOverrideCursor();
  if (!loaded) {
    QMessageBox::critical(this, "载入节点图失败", error);
  }
}

void NodeMapEditorPanel::saveNodeMaps()
{
  QString output = output_directory_edit_->text().trimmed();
  if (output.isEmpty()) {
    chooseOutputDirectory();
    output = output_directory_edit_->text().trimmed();
  }
  if (output.isEmpty()) {
    return;
  }

  const std::size_t unsafe_count = EditorModel::instance()->activeUnsafeEdgeCount();
  if (unsafe_count > 0) {
    const auto answer = QMessageBox::warning(
      this, "存在不安全手工边",
      QString("当前显示层有 %1 条红色不安全边。仍然保存吗？").arg(unsafe_count),
      QMessageBox::Save | QMessageBox::Cancel, QMessageBox::Cancel);
    if (answer != QMessageBox::Save) {
      return;
    }
  }

  QString error;
  QApplication::setOverrideCursor(Qt::WaitCursor);
  const bool saved = EditorModel::instance()->saveToDirectory(
    output.toStdString(), error);
  QApplication::restoreOverrideCursor();
  if (!saved) {
    QMessageBox::critical(this, "保存节点图失败", error);
  } else {
    QMessageBox::information(
      this, "保存完成",
      "已写入 distance_map、L1、L2 和 L3 文件。\n"
      "Theta* 的 node_map_yaml 指向该目录下的 nodemap_global.yaml。");
  }
}

void NodeMapEditorPanel::changeActiveLayer(int index)
{
  const auto layer = index == 1 ? EditorModel::Layer::LOCAL :
    index == 2 ? EditorModel::Layer::RAW : EditorModel::Layer::GLOBAL;
  EditorModel::instance()->setActiveLayer(layer);
}

void NodeMapEditorPanel::activateTool(const QString & class_id)
{
  auto * tool_manager = getDisplayContext()->getToolManager();
  for (int index = 0; index < tool_manager->numTools(); ++index) {
    auto * tool = tool_manager->getTool(index);
    if (tool && tool->getClassId() == class_id) {
      tool_manager->setCurrentTool(tool);
      return;
    }
  }
  auto * tool = tool_manager->addTool(class_id);
  if (tool) {
    tool_manager->setCurrentTool(tool);
  } else {
    showStatus("RViz 无法加载编辑工具，请检查插件是否已正确编译并 source");
  }
}

void NodeMapEditorPanel::activateMoveTool()
{
  activateTool(kMoveNodeToolId);
}

void NodeMapEditorPanel::activateEdgeTool()
{
  EditorModel::instance()->setManualEdgeSafetyCheck(
    manual_safety_check_->isChecked());
  activateTool(kEditEdgeToolId);
}

void NodeMapEditorPanel::rebuildDefaultConnections()
{
  const auto answer = QMessageBox::question(
    this, "恢复默认连接",
    "这会清除当前层所有人工添加和人工删除记录，并按连接半径、障碍物和净空重新生成。继续吗？",
    QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
  if (answer != QMessageBox::Yes) {
    return;
  }
  QString error;
  if (!EditorModel::instance()->rebuildActiveDefaultConnections(error)) {
    QMessageBox::warning(this, "无法重建", error);
  }
}

void NodeMapEditorPanel::refreshSummary()
{
  const auto * model = EditorModel::instance();
  if (!model->hasActiveLayer()) {
    summary_label_->setText("尚未生成或载入节点图");
    return;
  }
  const QString layer = model->activeLayer() == EditorModel::Layer::GLOBAL ? "L1 global" :
    model->activeLayer() == EditorModel::Layer::LOCAL ? "L2 local" : "L3 raw";
  summary_label_->setText(
    QString("%1：%2 个节点，%3 条连接，间距 %4 m，不安全边 %5")
    .arg(layer)
    .arg(model->activeNodeCount())
    .arg(model->activeEdgeCount())
    .arg(model->activeNodeSpacing(), 0, 'f', 3)
    .arg(model->activeUnsafeEdgeCount()));
  const bool editable = model->activeLayerEditable();
  move_node_button_->setEnabled(editable);
  edit_edge_button_->setEnabled(editable);
  rebuild_edges_button_->setEnabled(editable);
}

void NodeMapEditorPanel::showStatus(const QString & message)
{
  status_label_->setText(message);
}

void NodeMapEditorPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
  config.mapSetValue("source_map", source_map_edit_->text());
  config.mapSetValue("output_directory", output_directory_edit_->text());
  config.mapSetValue("active_layer", active_layer_combo_->currentIndex());
  config.mapSetValue("global_spacing", global_widgets_.spacing->value());
  config.mapSetValue("global_clearance", global_widgets_.clearance->value());
  config.mapSetValue("global_radius", global_widgets_.connection_radius->value());
  config.mapSetValue("global_offset_x", global_widgets_.offset_x->value());
  config.mapSetValue("global_offset_y", global_widgets_.offset_y->value());
  config.mapSetValue("local_spacing", local_widgets_.spacing->value());
  config.mapSetValue("local_clearance", local_widgets_.clearance->value());
  config.mapSetValue("local_radius", local_widgets_.connection_radius->value());
  config.mapSetValue("local_offset_x", local_widgets_.offset_x->value());
  config.mapSetValue("local_offset_y", local_widgets_.offset_y->value());
  config.mapSetValue("unknown_as_obstacle", unknown_obstacle_check_->isChecked());
  config.mapSetValue("manual_safety", manual_safety_check_->isChecked());
}

void NodeMapEditorPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
  QString text;
  int integer = 0;
  float number = 0.0F;
  bool boolean = false;
  if (config.mapGetString("source_map", &text)) {
    source_map_edit_->setText(text);
  }
  if (config.mapGetString("output_directory", &text)) {
    output_directory_edit_->setText(text);
  }
  if (config.mapGetInt("active_layer", &integer)) {
    active_layer_combo_->setCurrentIndex(std::clamp(integer, 0, 2));
  }
  auto loadDouble = [&config, &number](const char * key, QDoubleSpinBox * spin) {
      if (config.mapGetFloat(key, &number)) {
        spin->setValue(number);
      }
    };
  loadDouble("global_spacing", global_widgets_.spacing);
  loadDouble("global_clearance", global_widgets_.clearance);
  loadDouble("global_radius", global_widgets_.connection_radius);
  loadDouble("global_offset_x", global_widgets_.offset_x);
  loadDouble("global_offset_y", global_widgets_.offset_y);
  loadDouble("local_spacing", local_widgets_.spacing);
  loadDouble("local_clearance", local_widgets_.clearance);
  loadDouble("local_radius", local_widgets_.connection_radius);
  loadDouble("local_offset_x", local_widgets_.offset_x);
  loadDouble("local_offset_y", local_widgets_.offset_y);
  if (config.mapGetBool("unknown_as_obstacle", &boolean)) {
    unknown_obstacle_check_->setChecked(boolean);
  }
  if (config.mapGetBool("manual_safety", &boolean)) {
    manual_safety_check_->setChecked(boolean);
  }
}

}  // namespace nodemap_rviz_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nodemap_rviz_plugin::NodeMapEditorPanel, rviz_common::Panel)
