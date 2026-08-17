#include "nodemap_rviz_plugin/move_node_tool.hpp"

#include <OgreManualObject.h>
#include <OgreRenderOperation.h>
#include <OgreSceneManager.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "nodemap_rviz_plugin/editor_model.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

namespace nodemap_rviz_plugin
{
namespace
{

void drawRing(
  Ogre::ManualObject * object,
  float x, float y, float z, float radius,
  float red, float green, float blue)
{
  object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
  object->colour(red, green, blue, 1.0F);
  constexpr int kSegments = 24;
  for (int segment = 0; segment <= kSegments; ++segment) {
    const float angle =
      static_cast<float>(segment) / kSegments * 2.0F * static_cast<float>(M_PI);
    object->position(
      x + radius * std::cos(angle),
      y + radius * std::sin(angle), z);
  }
  object->end();
}

void drawSegment(
  Ogre::ManualObject * object,
  float first_x, float first_y, float second_x, float second_y,
  float red, float green, float blue)
{
  object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
  object->colour(red, green, blue, 1.0F);
  object->position(first_x, first_y, 0.20F);
  object->colour(red, green, blue, 1.0F);
  object->position(second_x, second_y, 0.20F);
  object->end();
}

}  // namespace

MoveNodeTool::MoveNodeTool()
{
  shortcut_key_ = 'm';
}

MoveNodeTool::~MoveNodeTool()
{
  if (!context_) {
    return;
  }
  auto * scene = context_->getSceneManager();
  if (preview_node_) {
    scene->destroySceneNode(preview_node_);
    preview_node_ = nullptr;
  }
  if (preview_object_) {
    scene->destroyManualObject(preview_object_);
    preview_object_ = nullptr;
  }
}

void MoveNodeTool::onInitialize()
{
  auto * scene = context_->getSceneManager();
  const std::string name =
    "nodemap_move_preview_" + std::to_string(reinterpret_cast<std::uintptr_t>(this));
  preview_object_ = scene->createManualObject(name);
  preview_object_->setDynamic(true);
  preview_node_ = scene->getRootSceneNode()->createChildSceneNode();
  preview_node_->attachObject(preview_object_);
  preview_node_->setVisible(false);
}

void MoveNodeTool::activate()
{
  dragging_ = false;
  selected_node_id_ = -1;
  if (!EditorModel::instance()->activeLayerEditable()) {
    setStatus("请先生成节点图并选择可编辑的 L1 或 L2");
    return;
  }
  setStatus("在节点附近按住左键拖动，松开后自动重算默认连接；右键退出");
}

void MoveNodeTool::deactivate()
{
  dragging_ = false;
  selected_node_id_ = -1;
  clearPreview();
}

int MoveNodeTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  auto * model = EditorModel::instance();
  if (!model->activeLayerEditable()) {
    return Render;
  }

  Ogre::Vector3 position;
  const bool has_position =
    context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, position);

  if (event.rightDown()) {
    clearPreview();
    return Render | Finished;
  }

  if (event.leftDown() && has_position) {
    const double snap_radius = std::clamp(
      model->activeNodeSpacing() * 0.40, 0.12, 0.45);
    selected_node_id_ = model->nearestActiveNode(
      position.x, position.y, snap_radius);
    if (selected_node_id_ < 0) {
      setStatus("附近没有节点，请更靠近节点中心点击");
      return Render;
    }
    dragging_ = true;
    updatePreview(position.x, position.y, true);
    setStatus(QString("正在拖动节点 %1，松开左键确认").arg(selected_node_id_));
    return Render;
  }

  if (event.type == QEvent::MouseMove && dragging_ && has_position) {
    updatePreview(position.x, position.y, true);
    return Render;
  }

  if (event.leftUp() && dragging_) {
    dragging_ = false;
    clearPreview();
    if (!has_position) {
      setStatus("无法获取释放位置，节点未移动");
      selected_node_id_ = -1;
      return Render;
    }
    QString error;
    if (!model->moveActiveNode(
        selected_node_id_, position.x, position.y, error))
    {
      setStatus(QString("移动失败：%1").arg(error));
    }
    selected_node_id_ = -1;
    return Render;
  }

  return Render;
}

void MoveNodeTool::updatePreview(double x, double y, bool valid_target)
{
  if (!preview_object_ || !preview_node_) {
    return;
  }
  preview_object_->clear();
  preview_node_->setVisible(true);
  const auto * layer = EditorModel::instance()->activeNodeMap();
  if (
    layer && selected_node_id_ >= 0 &&
    static_cast<std::size_t>(selected_node_id_) < layer->nodes.size())
  {
    const auto & original = layer->nodes[static_cast<std::size_t>(selected_node_id_)];
    drawRing(
      preview_object_, original.x, original.y, 0.19F, 0.18F,
      1.0F, 0.85F, 0.05F);
    drawSegment(
      preview_object_, original.x, original.y, x, y,
      1.0F, 0.85F, 0.05F);
  }
  drawRing(
    preview_object_, x, y, 0.21F, 0.22F,
    valid_target ? 0.10F : 1.0F,
    valid_target ? 1.0F : 0.05F, 0.15F);
}

void MoveNodeTool::clearPreview()
{
  if (preview_object_) {
    preview_object_->clear();
  }
  if (preview_node_) {
    preview_node_->setVisible(false);
  }
}

}  // namespace nodemap_rviz_plugin

PLUGINLIB_EXPORT_CLASS(nodemap_rviz_plugin::MoveNodeTool, rviz_common::Tool)
