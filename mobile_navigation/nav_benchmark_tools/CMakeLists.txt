cmake_minimum_required(VERSION 3.8)
project(nav_benchmark_tools)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/nbt DESTINATION lib/${PROJECT_NAME})
install(FILES resources/tutorial_template.html resources/curves_template.html DESTINATION share/${PROJECT_NAME}/resources)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_cli test/test_cli.py TIMEOUT 60)
  ament_add_pytest_test(test_brake_analyze test/test_brake_analyze.py TIMEOUT 120)
  ament_add_pytest_test(test_speed_tools test/test_speed_tools.py TIMEOUT 120)
  ament_add_pytest_test(test_pathnav_fake test/test_pathnav_fake.py TIMEOUT 180)
  ament_add_pytest_test(test_pathplan test/test_pathplan.py TIMEOUT 60)
endif()

ament_package()
