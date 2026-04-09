# Install script for directory: /home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/dex/tmp_fixworkspace/dexe_mobile_nav/install/mobile_perception")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/charging_pose_service_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/charging_pose_service_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/charging_pose_service_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mobile_perception" TYPE EXECUTABLE FILES "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/charging_pose_service_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/charging_pose_service_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/charging_pose_service_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/charging_pose_service_node"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/charging_pose_service_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/CMakeFiles/charging_pose_service_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/apriltag_detection_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/apriltag_detection_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/apriltag_detection_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mobile_perception" TYPE EXECUTABLE FILES "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/apriltag_detection_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/apriltag_detection_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/apriltag_detection_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/apriltag_detection_node"
         OLD_RPATH "/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/apriltag_detection_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/CMakeFiles/apriltag_detection_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/mobile_control_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/mobile_control_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/mobile_control_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mobile_perception" TYPE EXECUTABLE FILES "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/mobile_control_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/mobile_control_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/mobile_control_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/mobile_control_node"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mobile_perception/mobile_control_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/CMakeFiles/mobile_control_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mobile_perception" TYPE PROGRAM FILES
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/detect_apriltags.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/detect_apriltag_nostop.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/detect_apriltags_service.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/control_robot.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/nav_then_precise.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/nav_then_precise_bt.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/goal_controller.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/stop_robot.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/spin_robot.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/undocking.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/control_robot_simulation.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/goal_controller_simulation.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/nav_goal_publisher.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/nav_goal_publisher_loop.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/pub_current_position.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/pub_goal_loop_office.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/create_charging_pose.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/nav_then_precise_bt.py"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/scripts/nav_goal_server.py"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mobile_perception" TYPE DIRECTORY FILES
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/launch"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/config"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/images"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/mobile_perception")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/mobile_perception")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mobile_perception/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mobile_perception/environment" TYPE FILE FILES "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mobile_perception/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mobile_perception/environment" TYPE FILE FILES "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mobile_perception" TYPE FILE FILES "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mobile_perception" TYPE FILE FILES "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mobile_perception" TYPE FILE FILES "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mobile_perception" TYPE FILE FILES "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mobile_perception" TYPE FILE FILES "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/ament_cmake_index/share/ament_index/resource_index/packages/mobile_perception")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mobile_perception/cmake" TYPE FILE FILES "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mobile_perception/cmake" TYPE FILE FILES "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mobile_perception/cmake" TYPE FILE FILES
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/ament_cmake_core/mobile_perceptionConfig.cmake"
    "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/ament_cmake_core/mobile_perceptionConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mobile_perception" TYPE FILE FILES "/home/dex/tmp_fixworkspace/dexe_mobile_nav/mobile_perception/package.xml")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/dex/tmp_fixworkspace/dexe_mobile_nav/build/mobile_perception/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
