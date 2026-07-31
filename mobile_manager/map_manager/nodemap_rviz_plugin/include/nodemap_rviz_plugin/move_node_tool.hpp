#ifndef NODEMAP_RVIZ_PLUGIN__MOVE_NODE_TOOL_HPP_
#define NODEMAP_RVIZ_PLUGIN__MOVE_NODE_TOOL_HPP_

#include <OgreManualObject.h>
#include <OgreSceneNode.h>

#include "rviz_common/tool.hpp"

namespace nodemap_rviz_plugin
{

class MoveNodeTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  MoveNodeTool();
  ~MoveNodeTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private:
  void updatePreview(double x, double y, bool valid_target);
  void clearPreview();

  Ogre::ManualObject * preview_object_{nullptr};
  Ogre::SceneNode * preview_node_{nullptr};
  int selected_node_id_{-1};
  bool dragging_{false};
};

}  // namespace nodemap_rviz_plugin

#endif  // NODEMAP_RVIZ_PLUGIN__MOVE_NODE_TOOL_HPP_
