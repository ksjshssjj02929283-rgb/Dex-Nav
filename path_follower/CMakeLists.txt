cmake_minimum_required(VERSION 3.8)
project(path_follower)

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
# Dependencies: ROS2 Humble ecosystem libraries only.
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ruckig REQUIRED)  # jerk 限制轨迹生成（路径下发时的整段全局速度规划）

set(THIS_PKG_DEPENDS
        rclcpp
        rclcpp_action
        rclcpp_components
        nav_msgs
        geometry_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        angles
        nav_interfaces
        std_srvs
)

# ---------------------------------------------------------------------------
# Library: pure trajectory-tracking core.
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
        src/pid_controller.cpp
        src/velocity_planner.cpp
        src/ruckig_velocity_planner.cpp
        src/velocity_step_model.cpp
        src/path_follower.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ruckig::ruckig)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PKG_DEPENDS})

# ---------------------------------------------------------------------------
# Component node (composable) + standalone executable.
# ---------------------------------------------------------------------------
rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN "path_follower::PathFollowerNode"
        EXECUTABLE path_follower_node
)

# ---------------------------------------------------------------------------
# Install.
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

# Python 脚本：贝塞尔轨迹生成器 / TrackingPath 轨迹生成器（测试用）
install(PROGRAMS scripts/bezier_path_generator.py
              scripts/tracking_path_generator.py
        DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PKG_DEPENDS})
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_export_libraries(${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
