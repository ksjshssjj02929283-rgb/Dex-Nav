cmake_minimum_required(VERSION 3.8)
project(nav2_schedule_planner)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()
if (NOT CMAKE_CXX_STANDARD_REQUIRED)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# ---------------------------------------------------------------------------
# 依赖
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(THIS_PKG_DEPENDS
        rclcpp
        rclcpp_components
        rclcpp_action
        nav_msgs
        nav2_msgs
        geometry_msgs
        nav_interfaces
        std_msgs
        std_srvs
)

# ---------------------------------------------------------------------------
# 库：路径段采样器 + 执行节点
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
        src/path_segment_sampler.cpp
        src/segment_merger.cpp
        src/schedule_planner_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PKG_DEPENDS})

# ---------------------------------------------------------------------------
# Component node (composable) + standalone executable（默认 SingleThreadedExecutor，
# execute() 跑在专用线程，不占执行器线程）。
# ---------------------------------------------------------------------------
rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN "schedule_planner::SchedulePlannerNode"
        EXECUTABLE schedule_planner_node
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
	    scripts/graph_nav.py
        scripts/random_navigate_through_graph_points.py
        scripts/plot_move_record.py
        DESTINATION lib/${PROJECT_NAME}
)

# 路网同步工具：sync_roadnet.py 会按同目录找 map_convert_nav2tc.py，两个必须装在一起
install(PROGRAMS
        scripts/map_sync/sync_roadnet.py
        scripts/map_sync/map_convert_nav2tc.py
        DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PKG_DEPENDS})
ament_export_libraries(${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
