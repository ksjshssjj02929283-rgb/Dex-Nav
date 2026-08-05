cmake_minimum_required(VERSION 3.5)
project(nav2_mppi_controller)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
)

set(dependencies_pkgs
  rclcpp
  nav2_common
  pluginlib
  tf2
  geometry_msgs
  visualization_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()

nav2_package()

include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-mno-avx512f" COMPILER_SUPPORTS_AVX512)
check_cxx_compiler_flag("-msse4.2" COMPILER_SUPPORTS_SSE4)
check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)

if(COMPILER_SUPPORTS_AVX512)
  add_compile_options(-mno-avx512f)
endif()

if(COMPILER_SUPPORTS_SSE4)
  add_compile_options(-msse4.2)
endif()

if(COMPILER_SUPPORTS_AVX2)
  add_compile_options(-mavx2)
endif()
# 使用标准的 CPU 浮点运算 (遵循 IEEE 754 标准，移除 -ffast-math 与 -ffp-contract=fast)
add_compile_options(-O3 -mtune=generic)

find_program(NVCC_EXECUTABLE nvcc HINTS /usr/bin /usr/local/cuda/bin /usr/local/cuda-12.6/bin /usr/local/cuda-12.0/bin /usr/local/cuda-11.8/bin /usr/local/cuda-11.5/bin)
if(NVCC_EXECUTABLE)
  set(CMAKE_CUDA_COMPILER ${NVCC_EXECUTABLE})

  # 自动判定目标平台 (Jetson Orin ARM64 vs x86_64 Desktop)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "" OR CMAKE_CUDA_ARCHITECTURES STREQUAL "native" OR CMAKE_CUDA_ARCHITECTURES STREQUAL "auto")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR EXISTS "/etc/nv_tegra_release" OR EXISTS "/proc/device-tree/model")
      message(STATUS "--------------------------------------------------------")
      message(STATUS "[MPPI CUDA] Target Hardware Detected: NVIDIA Jetson Orin (ARM64)")
      message(STATUS "[MPPI CUDA] Setting CUDA Architecture: sm_87")
      message(STATUS "--------------------------------------------------------")
      set(CMAKE_CUDA_ARCHITECTURES "87")
      add_compile_definitions(PLATFORM_JETSON_ORIN)
    else()
      message(STATUS "--------------------------------------------------------")
      message(STATUS "[MPPI CUDA] Target Hardware Detected: x86_64 Desktop / Server")
      message(STATUS "[MPPI CUDA] Setting CUDA Architectures: sm_75;sm_80;sm_86")
      message(STATUS "--------------------------------------------------------")
      set(CMAKE_CUDA_ARCHITECTURES "75;80;86")
      add_compile_definitions(PLATFORM_X86_64)
    endif()
  else()
    message(STATUS "[MPPI CUDA] User specified custom CMAKE_CUDA_ARCHITECTURES: ${CMAKE_CUDA_ARCHITECTURES}")
  endif()
  
  get_directory_property(CURRENT_DIR_COMPILE_OPTIONS COMPILE_OPTIONS)
  string(REPLACE "-Wdeprecated" "" CLEAN_COMPILE_OPTIONS "${CURRENT_DIR_COMPILE_OPTIONS}")
  string(REPLACE "-Werror" "" CLEAN_COMPILE_OPTIONS "${CLEAN_COMPILE_OPTIONS}")
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "${CLEAN_COMPILE_OPTIONS}")

  enable_language(CUDA)
  add_definitions(-DMPPI_CUDA_ENABLED)

  add_library(mppi_cuda_backend SHARED src/cuda/cuda_optimizer_kernels.cu)
  set_target_properties(mppi_cuda_backend PROPERTIES
    CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}"
    CUDA_SEPARABLE_COMPILATION ON
  )
  
  # 普通 GPU 浮点运算设置 (遵循标准 IEEE 754 规范，精确除法与开方)
  target_compile_options(mppi_cuda_backend PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:
      --ftz=false
      --prec-div=true
      --prec-sqrt=true
    >
  )

  target_include_directories(mppi_cuda_backend PUBLIC include)
  set(CUDA_LINK_LIBS mppi_cuda_backend)
  set(CUDA_SOURCES src/cuda/cuda_optimizer.cpp)
  message(STATUS "[MPPI CMake] Found NVCC compiler at ${NVCC_EXECUTABLE}. CUDA GPU Acceleration ENABLED! Target CUDA Architecture: ${CMAKE_CUDA_ARCHITECTURES}")
else()
  set(CUDA_SOURCES src/cuda/cuda_optimizer_stub.cpp)
  set(CUDA_LINK_LIBS "")
  message(STATUS "[MPPI CMake] NVCC compiler NOT found. CUDA disabled.")
endif()

add_library(mppi_controller SHARED
  src/controller.cpp
  src/optimizer.cpp
  src/critic_manager.cpp
  src/trajectory_visualizer.cpp
  src/path_handler.cpp
  src/parameters_handler.cpp
  src/noise_generator.cpp
  ${CUDA_SOURCES}
)

add_library(mppi_critics SHARED
  src/critics/obstacles_critic.cpp
  src/critics/cost_critic.cpp
  src/critics/goal_critic.cpp
  src/critics/goal_lateral_critic.cpp
  src/critics/goal_longitudinal_critic.cpp
  src/critics/goal_angle_critic.cpp
  src/critics/path_align_critic.cpp
  src/critics/path_align_legacy_critic.cpp
  src/critics/path_follow_critic.cpp
  src/critics/path_angle_critic.cpp
  src/critics/prefer_forward_critic.cpp
  src/critics/twirling_critic.cpp
  src/critics/constraint_critic.cpp
  src/critics/velocity_deadband_critic.cpp
  src/critics/linear_velocity_critic.cpp
  src/critics/angular_velocity_critic.cpp
)

set(libraries mppi_controller mppi_critics)

foreach(lib IN LISTS libraries)
  target_compile_options(${lib} PUBLIC -fconcepts)
  target_link_libraries(${lib} Eigen3::Eigen ${CUDA_LINK_LIBS})
  ament_target_dependencies(${lib} ${dependencies_pkgs})
endforeach()

target_link_libraries(mppi_critics mppi_controller)

install(TARGETS mppi_controller mppi_critics ${CUDA_LINK_LIBS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
  # add_subdirectory(benchmark)
endif()

ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies_pkgs})
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(nav2_core mppic.xml)
pluginlib_export_plugin_description_file(nav2_mppi_controller critics.xml)

ament_package()
