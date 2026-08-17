#include "nodemap_rviz_plugin/edit_edge_tool.hpp"

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
  float x, float y, float radius,
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
      y + radius * std::sin(angle), 0.21F);
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

EditEdgeTool::EditEdgeTool()
{
  shortcut_key_ = 'e';
}

EditEdgeTool::~EditEdgeTool()
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

void EditEdgeTool::onInitialize()
{
  auto * scene = context_->getSceneManager();
  const std::string name =
    "nodemap_edge_preview_" + std::to_string(reinterpret_cast<std::uintptr_t>(this));
  preview_object_ = scene->createManualObject(name);
  preview_object_->setDynamic(true);
  preview_node_ = scene->getRootSceneNode()->createChildSceneNode();
  preview_node_->attachObject(preview_object_);
  preview_node_->setVisible(false);
}

void EditEdgeTool::activate()
{
  first_node_id_ = -1;
  if (!EditorModel::instance()->activeLayerEditable()) {
    setStatus("请先生成节点图并选择可编辑的 L1 或 L2");
    return;
  }
  setStatus("左键选择第一个节点，再左键选择第二个节点；已有边会删除，无边会添加；右键退出");
}

void EditEdgeTool::deactivate()
{
  first_node_id_ = -1;
  clearPreview();
}

int EditEdgeTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  auto * model = EditorModel::instance();
  if (!model->activeLayerEditable()) {
    return Render;
  }

  Ogre::Vector3 position;
  const bool has_position =
    context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, position);

  if (event.rightDown()) {
    if (first_node_id_ >= 0) {
      first_node_id_ = -1;
      clearPreview();
      setStatus("已取消第一个端点；再次右键退出");
      return Render;
    }
    clearPreview();
    return Render | Finished;
  }

  if (event.type == QEvent::MouseMove && has_position) {
    updatePreview(position.x, position.y);
    return Render;
  }

  if (event.leftDown() && has_position) {
    const double snap_radius = std::clamp(
      model->activeNodeSpacing() * 0.40, 0.12, 0.45);
    const int selected = model->nearestActiveNode(
      position.x, position.y, snap_radius);
    if (selected < 0) {
      setStatus("附近没有节点，请更靠近节点中心点击");
      return Render;
    }

    if (first_node_id_ < 0) {
      first_node_id_ = selected;
      setStatus(QString("已选择节点 %1，请选择第二个节点").arg(first_node_id_));
      updatePreview(position.x, position.y);
      return Render;
    }

    if (selected == first_node_id_) {
      setStatus("第二个端点不能与第一个端点相同");
      return Render;
    }

    QString error;
    const int previous_first = first_node_id_;
    if (!model->toggleActiveEdge(
        first_node_id_, selected, model->manualEdgeSafetyCheck(), error))
    {
      setStatus(QString("修改连接失败：%1").arg(error));
      return Render;
    }

    // 链式编辑：第二个端点自动成为下一条边的第一个端点，连续连点更方便。
    first_node_id_ = selected;
    setStatus(
      QString("连接 %1 ↔ %2 已切换；节点 %2 作为下一条边起点，右键取消")
      .arg(previous_first).arg(selected));
    updatePreview(position.x, position.y);
    return Render;
  }

  return Render;
}

void EditEdgeTool::updatePreview(double x, double y)
{
  if (!preview_object_ || !preview_node_) {
    return;
  }
  preview_object_->clear();
  preview_node_->setVisible(true);

  auto * model = EditorModel::instance();
  const auto * layer = model->activeNodeMap();
  const double snap_radius = std::clamp(
    model->activeNodeSpacing() * 0.40, 0.12, 0.45);
  const int hovered = model->nearestActiveNode(x, y, snap_radius);

  if (
    first_node_id_ >= 0 && layer &&
    static_cast<std::size_t>(first_node_id_) < layer->nodes.size())
  {
    const auto & first = layer->nodes[static_cast<std::size_t>(first_node_id_)];
    drawRing(preview_object_, first.x, first.y, 0.22F, 0.10F, 1.0F, 0.30F);
    drawSegment(preview_object_, first.x, first.y, x, y, 0.15F, 0.85F, 1.0F);
  }

  if (
    hovered >= 0 && layer &&
    static_cast<std::size_t>(hovered) < layer->nodes.size())
  {
    const auto & node = layer->nodes[static_cast<std::size_t>(hovered)];
    const bool will_remove =
      first_node_id_ >= 0 && model->activeEdgeExists(first_node_id_, hovered);
    drawRing(
      preview_object_, node.x, node.y, 0.24F,
      will_remove ? 1.0F : 1.0F,
      will_remove ? 0.10F : 0.85F,
      will_remove ? 0.10F : 0.05F);
  }
}

void EditEdgeTool::clearPreview()
{
  if (preview_object_) {
    preview_object_->clear();
  }
  if (preview_node_) {
    preview_node_->setVisible(false);
  }
}

}  // namespace nodemap_rviz_plugin

PLUGINLIB_EXPORT_CLASS(nodemap_rviz_plugin::EditEdgeTool, rviz_common::Tool)
