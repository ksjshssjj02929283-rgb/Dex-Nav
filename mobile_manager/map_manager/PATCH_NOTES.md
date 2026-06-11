# Map Manager ROS2 修复说明

## 问题
当前map_manager在加载地图后会尝试通知所有相关节点（nav_point_manager, mission_manager, move_base等）。
但是：
1. 服务名称不匹配：期望 `/nav_point_manager/set_map_changed`，实际是 `/set_map_changed`
2. 如果任何一个节点服务不可用，整个LoadMap会失败

## 建议修复

### 方案1：修改服务名称映射（推荐）
在`SetActivateSignalToNode`中添加特殊处理：

```cpp
bool MapManagerROS::SetActivateSignalToNode(const std::string& node_name)
{
  // 构建服务名称
  std::string service_name;
  if (node_name == "nav_point_manager") {
    // nav_point_manager的服务在根命名空间
    service_name = "/set_map_changed";
  } else {
    // 其他节点使用命名空间
    service_name = "/" + node_name + "/set_map_changed";
  }
  
  auto client = this->create_client<std_srvs::srv::Trigger>(service_name);
  if (!client->wait_for_service(1s)) {
    // 改为警告而不是错误，允许继续
    RCLCPP_WARN(this->get_logger(), "%s not exists, skipping", service_name.c_str());
    return true;  // 返回true允许继续
  }
  // ... 后续代码
}
```

### 方案2：修改ActivateCurrentMap（更宽松）
让失败的服务调用不影响整体：

```cpp
bool MapManagerROS::ActivateCurrentMap()
{
  const std::vector<std::string> nodes = {
    "nav_point_manager",
    "mission_manager",
    "move_base",
    "docking_base",
    "localization_node"
  };
  
  bool any_success = false;
  for (const auto& name : nodes) {
    if (SetActivateSignalToNode(name)) {
      any_success = true;
    } else {
      // 只记录警告，不影响整体
      RCLCPP_WARN(this->get_logger(), "Failed to notify %s, but continuing", name.c_str());
    }
  }
  
  // 只要有一个成功就算成功，或者允许全部失败也成功
  return true;  // 或者 return any_success;
}
```

## 当前workaround

测试脚本已经可以工作，虽然LoadMap返回false，但是：
1. 地图文件已经成功加载
2. 地图数据已经发布到/map话题
3. 可以继续使用地图进行测试

后续如需完整功能，请应用上述修复方案之一。

