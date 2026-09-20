cmake_minimum_required(VERSION 3.8)
project(nav_interfaces)

# 编译配置
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1) 基础依赖（ROS 2）
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2) 依赖的消息/动作包
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 3) 声明要生成的消息
set(MSG_FILES
  "msg/Path.msg"
  "msg/PathPose.msg"
  "msg/AreaInfo.msg"
  "msg/AreaInfoArray.msg"
  "msg/DTLane.msg"
  "msg/Lane.msg"
  "msg/LaneArray.msg"
  "msg/Waypoint.msg"
  "msg/WaypointState.msg"
  "msg/NavPoint.msg"
  "msg/DockingInfo.msg"
  "msg/Task.msg"
  "msg/TaskGroup.msg"
  "msg/BatteryInfo.msg"
  "msg/Line.msg"
  "msg/LineArray.msg"
  "msg/LineMapData.msg"
  "msg/LineMapDataInfo.msg"
  "msg/TargetInfo.msg"
  "msg/TargetMapInfo.msg"
  "msg/LocalizationState.msg"
  "msg/MapState.msg"
  "msg/MappingState.msg"
  "msg/RobotStatus.msg"
  "msg/NavigationStatus.msg"
  "msg/NavSystemStatus.msg"
  "msg/ChangeTopology.msg"
  "msg/TopologyLine.msg"
  "msg/Route.msg"
  "msg/RouteEndpoint.msg"
  "msg/RouteJunction.msg"
  "msg/RouteEdge.msg"
  "msg/RoutePlanSegment.msg"
  "msg/RoutePlan.msg"
  "msg/RoomBehavior.msg"
  "msg/RoomInfo.msg"
  "msg/RoomInfoArray.msg"
  "msg/TrajectoryPoint.msg"
  "msg/RobotTrajectory.msg"
  "msg/Story.msg"
  "msg/RobotStories.msg"
  "msg/TaskProgress.msg"
  "msg/PathSegment.msg"
  "msg/TrackingPath.msg"
  "msg/ObstacleMonitorStatus.msg"
)

# 4) 声明要生成的服务
set(SRV_FILES
  "srv/AddTask.srv"
  "srv/AddTaskGroup.srv"
  "srv/AddNavPoint.srv"
  "srv/SampleNavPoint.srv"
  "srv/RemoveNavPoint.srv"
  "srv/UpdateNavPoint.srv"
  "srv/GetNavPoint.srv"
  "srv/GetAllNavPoints.srv"
  "srv/AddRoute.srv"
  "srv/RemoveRoute.srv"
  "srv/GetRoute.srv"
  "srv/GetAllRoutes.srv"
  "srv/NavToPoint.srv"
  "srv/GetCurrentTaskInfo.srv"
  "srv/SetPose.srv"
  "srv/GetStatus.srv"
  "srv/SetCommandBoard.srv"
  "srv/SetMapName.srv"
  "srv/GetMapList.srv"
  "srv/LoadMap.srv"
  "srv/DeleteMap.srv"
  "srv/RenameMap.srv"
  "srv/GetMapInfo.srv"
  "srv/GetString.srv"
  "srv/SaveMap.srv"
  "srv/AddAreaInfo.srv"
  "srv/GetAreaInfoList.srv"
  "srv/GetAllAreas.srv"
  "srv/RemoveAreaInfo.srv"
  "srv/UpdateAreaInfo.srv"
  "srv/NavThroughPosesFile.srv"
  "srv/LoadPoint.srv"
  "srv/AddRouteJunction.srv"
  "srv/RemoveRouteJunction.srv"
  "srv/AddRouteEdge.srv"
  "srv/RemoveRouteEdge.srv"
  "srv/UpdateRouteEdge.srv"
  "srv/GetRouteGraph.srv"
  "srv/TransformAssets.srv"
  "srv/ResetMapOriginByXYYaw.srv"
  "srv/AddRoom.srv"
  "srv/RemoveRoom.srv"
  "srv/UpdateRoom.srv"
  "srv/GetAllRooms.srv"
  "srv/GetRoomByName.srv"
  "srv/GetRoomAtPose.srv"
  "srv/SaveAssets.srv"
  "srv/RollbackMap.srv"
  "srv/ResolveNavPoints.srv"
  "srv/SetPolygonPadding.srv"
  "srv/SetNavigationBackend.srv"
)

# 5) 声明要生成的动作
set(ACTION_FILES
  "action/NavigateToPoint.action"
  "action/NavigateRoute.action"
  "action/NavigateThroughGraphPoints.action"
  "action/ExecuteMissionBt.action"
  "action/PreciseControl.action"
  "action/SteeringAngleControl.action"
)

# 6) 生成接口（取代 ROS 1 的 add_message_files / add_service_files / generate_messages）
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    action_msgs
    actionlib_msgs
    builtin_interfaces
)

# 7) 导出运行期依赖
ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  nav_msgs
  action_msgs
  actionlib_msgs
  builtin_interfaces
)

ament_package()
